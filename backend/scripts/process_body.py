import os
import argparse
import logging
from pathlib import Path
import trimesh
import numpy as np
import warnings

from scripts.geometry_checks import check_geometry
from scripts.physics_checks import check_physics

log = logging.getLogger(__name__)

def evaluate_mesh(mesh_path: str | Path) -> tuple[bool, dict]:
    """
    Evaluates the mesh using the checking scripts.
    Returns (is_ready, report).
    """
    is_ready = True
    report = {
        'geometry': check_geometry(str(mesh_path)),
        'physics': check_physics(str(mesh_path))
    }
    
    # Analyze if we need fixing
    if report['geometry']['critical']:
        is_ready = False
        
    # Scale or disconnected warnings
    geo_warnings = report['geometry']['warnings']
    if any("unreferenced vertices" in w for w in geo_warnings) or \
       any("centimeters" in w for w in geo_warnings):
        is_ready = False
        
    phys_warnings = report['physics']['warnings']
    if any("sharp edges" in w for w in phys_warnings):
        # We try to fix sharp edges with merge + decimate
        is_ready = False
        
    # We tolerate thin geometry for now since the user requested to keep limbs
    return is_ready, report

def smart_process(input_path: str | Path, target_height: float = 1.75, decimate_target: int = 5000, force: bool = False) -> Path:
    """
    Loads, evaluates, and fixes the GLB mesh if necessary.
    Returns the path to the physics-ready GLB.
    """
    input_path = Path(input_path)
    output_path = input_path.parent / f"{input_path.stem}_physics.glb"
    
    if output_path.exists() and not force:
        log.info(f"Physics proxy {output_path} already exists. Evaluating...")
        is_ready, report = evaluate_mesh(output_path)
        if is_ready:
            log.info("Proxy is verified and ready for physics.")
            return output_path
        else:
            log.warning("Existing proxy has issues. Reprocessing...")
            
    log.info(f"Evaluating raw source mesh: {input_path}")
    is_ready, report = evaluate_mesh(input_path)
    
    if is_ready and not force:
        log.info("Source mesh is already perfect for physics simulation. Using as-is.")
        # We can just copy it or symlink it, or export it to the physics path
        mesh = trimesh.load(str(input_path), force='mesh')
        mesh.export(str(output_path))
        return output_path

    log.info("Processing mesh to resolve physics constraints...")
    mesh = trimesh.load(str(input_path), force='mesh')
    
    # 1. Weld vertices
    log.info("Merging disconnected vertices...")
    mesh.merge_vertices()
    
    # 2. Fix Scale
    bounds = mesh.bounds
    min_y = bounds[0][1]
    max_y = bounds[1][1]
    height = max_y - min_y
    if height > 50:
        log.info("Mesh appears to be in centimeters. Scaling to meters.")
        mesh.apply_scale(0.01)
        # re-evaluate bounds
        bounds = mesh.bounds
        min_y = bounds[0][1]
        max_y = bounds[1][1]
        height = max_y - min_y
        
    if abs(height - target_height) > 0.1:
        scale = target_height / height
        mesh.apply_scale(scale)
        log.info(f"Scaled to target height: {target_height}m")
        
    # Translate feet to Y=0
    min_y_scaled = mesh.bounds[0][1]
    mesh.apply_translation([0.0, -min_y_scaled, 0.0])
    
    # 3. Decimate
    if len(mesh.vertices) > 10000:
        try:
            log.info(f"High vertex count detected ({len(mesh.vertices)} > 10000). Decimating to {decimate_target} faces...")
            mesh = mesh.simplify_quadric_decimation(face_count=decimate_target)
        except Exception as e:
            log.warning(f"Decimation failed: {e}")
    else:
        log.info(f"Vertex count ({len(mesh.vertices)}) is under 10,000. Skipping decimation.")
            
    # 4. Remove degeneracies added by decimation
    areas = mesh.area_faces
    valid_mask = areas > 1e-10
    n_degenerate = int((~valid_mask).sum())
    if n_degenerate > 0:
        log.info(f"Removing {n_degenerate} degenerate triangles...")
        mesh.update_faces(valid_mask)
        mesh.remove_unreferenced_vertices()
        
    # 5. Fix Normal Orientation
    log.info("Recalculating outward normals...")
    mesh.fix_normals()
    
    # Export
    log.info(f"Saving physics proxy to {output_path}...")
    mesh.export(str(output_path))
    
    # Post-processing verification
    log.info("Verifying processed mesh...")
    final_ready, final_report = evaluate_mesh(output_path)
    
    if not final_ready:
        log.warning("Post-processing completed, but issues remain! Advanced intervention in Blender might be required.")
        log.warning(f"Geometry criticals: {final_report['geometry']['critical']}")
        log.warning(f"Physics warnings: {final_report['physics']['warnings']}")
    else:
        log.info("Post-processing successful. Proxy is certified for physics loop.")
        
    return output_path

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    parser = argparse.ArgumentParser()
    parser.add_argument("mesh_path", help="Path to input GLB/OBJ file")
    parser.add_argument("--force", action="store_true", help="Force reprocessing even if proxy exists")
    args = parser.parse_args()
    smart_process(args.mesh_path, force=args.force)
