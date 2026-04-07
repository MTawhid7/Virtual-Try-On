"""
Body Profile Analyzer v3 — Robust body mesh profiling for garment construction.

Uses a fixed torso X-limit of ±0.20m (determined from Blender wireframe analysis)
and median smoothing to produce stable measurements from sparse meshes.

Usage:
    python -m scripts.analyze_body

Output:
    data/bodies/mannequin_profile.json
"""

import json
from pathlib import Path

import numpy as np
import trimesh


# Fixed torso X-limit: determined by visual inspection of Blender wireframe.
# The torso core extends to ±0.17m at waist, ±0.20m at chest/shoulder.
# Arms begin at X ≈ ±0.20m. Using ±0.20m captures the full torso.
TORSO_X_LIMIT = 0.20


def analyze_body(
    mesh_path: str = "data/bodies/mannequin_physics.glb",
    output_path: str = "data/bodies/mannequin_profile.json",
    n_slices: int = 60,
) -> dict:
    print("═══════════════════════════════════════════════════")
    print("  Body Profile Analyzer v3 (fixed torso X-limit)")
    print("═══════════════════════════════════════════════════\n")

    mesh = trimesh.load(mesh_path, force="mesh")
    verts = np.array(mesh.vertices, dtype=np.float64)

    print(f"  Mesh: {len(verts)} vertices, {len(mesh.faces)} faces")
    print(f"  Torso X limit: ±{TORSO_X_LIMIT:.2f}m\n")

    y_min_v, y_max_v = float(verts[:, 1].min()), float(verts[:, 1].max())

    bounds = {
        "x_min": round(float(verts[:, 0].min()), 4),
        "x_max": round(float(verts[:, 0].max()), 4),
        "y_min": round(y_min_v, 4),
        "y_max": round(y_max_v, 4),
        "z_min": round(float(verts[:, 2].min()), 4),
        "z_max": round(float(verts[:, 2].max()), 4),
        "height": round(y_max_v - y_min_v, 4),
    }

    # ─── Raw slices ───
    heights = np.linspace(y_min_v + 0.03, y_max_v - 0.03, n_slices)
    band = 0.02

    raw = []
    for y in heights:
        all_mask = (verts[:, 1] >= y - band) & (verts[:, 1] <= y + band)
        if all_mask.sum() < 3:
            continue

        all_v = verts[all_mask]
        torso_v = all_v[np.abs(all_v[:, 0]) <= TORSO_X_LIMIT]
        arm_v = all_v[np.abs(all_v[:, 0]) > TORSO_X_LIMIT]

        if len(torso_v) < 3:
            continue

        raw.append({
            "y": float(y),
            "width": float(torso_v[:, 0].max() - torso_v[:, 0].min()),
            "depth": float(torso_v[:, 2].max() - torso_v[:, 2].min()),
            "xl": float(torso_v[:, 0].min()),
            "xr": float(torso_v[:, 0].max()),
            "zf": float(torso_v[:, 2].max()),
            "zb": float(torso_v[:, 2].min()),
            "has_arms": len(arm_v) >= 3,
            "n_verts": int(all_mask.sum()),
            "total_width": float(all_v[:, 0].max() - all_v[:, 0].min()),
            "arm_xl": float(arm_v[:, 0].min()) if len(arm_v) >= 2 else None,
            "arm_xr": float(arm_v[:, 0].max()) if len(arm_v) >= 2 else None,
        })

    # ─── Median smoothing (window=5) ───
    for field in ["width", "depth", "zf", "zb", "xl", "xr"]:
        vals = np.array([s[field] for s in raw])
        smoothed = vals.copy()
        for i in range(len(vals)):
            lo, hi = max(0, i - 2), min(len(vals), i + 3)
            smoothed[i] = np.median(vals[lo:hi])
        for i, s in enumerate(raw):
            s[field] = float(smoothed[i])

    # ─── Build cross-sections with circumference ───
    sections = []
    for s in raw:
        a, b = s["width"] / 2, s["depth"] / 2
        circ = float(np.pi * (3 * (a + b) - np.sqrt((3 * a + b) * (a + 3 * b)))) if a > 0.01 and b > 0.01 else 0.0

        section = {
            "y": round(s["y"], 4),
            "torso": {
                "width": round(s["width"], 4),
                "depth": round(s["depth"], 4),
                "circumference": round(circ, 4),
                "center_x": round((s["xl"] + s["xr"]) / 2, 4),
                "center_z": round((s["zf"] + s["zb"]) / 2, 4),
                "x_left": round(s["xl"], 4),
                "x_right": round(s["xr"], 4),
                "z_front": round(s["zf"], 4),
                "z_back": round(s["zb"], 4),
            },
            "has_arms": s["has_arms"],
            "total_width": round(s["total_width"], 4),
            "n_verts": s["n_verts"],
        }
        if s["arm_xl"] is not None:
            section["arms"] = {
                "x_min": round(s["arm_xl"], 4),
                "x_max": round(s["arm_xr"], 4),
            }
        sections.append(section)

    # ─── Landmark detection ───
    print("  Detecting landmarks...")
    ys = np.array([s["y"] for s in sections])
    ws = np.array([s["torso"]["width"] for s in sections])
    ds = np.array([s["torso"]["depth"] for s in sections])

    landmarks = {}

    def best(target_y):
        return sections[np.argmin(np.abs(ys - target_y))]

    # Chest: widest in Y=1.05-1.35
    m = (ys >= 1.05) & (ys <= 1.35)
    if m.any():
        i = np.where(m)[0][np.argmax(ws[m])]
        landmarks["chest_y"] = round(float(ys[i]), 4)
        landmarks["chest_width"] = round(float(ws[i]), 4)
        landmarks["chest_depth"] = round(float(ds[i]), 4)
        landmarks["chest_circumference"] = round(sections[i]["torso"]["circumference"], 4)
        landmarks["chest_z_front"] = round(sections[i]["torso"]["z_front"], 4)
        landmarks["chest_z_back"] = round(sections[i]["torso"]["z_back"], 4)

    # Waist: narrowest in Y=0.85-1.05
    m = (ys >= 0.85) & (ys <= 1.05)
    if m.any():
        i = np.where(m)[0][np.argmin(ws[m])]
        landmarks["waist_y"] = round(float(ys[i]), 4)
        landmarks["waist_width"] = round(float(ws[i]), 4)
        landmarks["waist_depth"] = round(float(ds[i]), 4)
        landmarks["waist_circumference"] = round(sections[i]["torso"]["circumference"], 4)

    # Hip: widest in Y=0.80-0.92 (garment-relevant hip = fullest buttock, not iliac crest)
    m = (ys >= 0.80) & (ys <= 0.92)
    if m.any():
        i = np.where(m)[0][np.argmax(ws[m])]
        landmarks["hip_y"] = round(float(ys[i]), 4)
        landmarks["hip_width"] = round(float(ws[i]), 4)
        landmarks["hip_depth"] = round(float(ds[i]), 4)
        landmarks["hip_circumference"] = round(sections[i]["torso"]["circumference"], 4)

    # Armpit: where the arm physically separates from the torso.
    # Scan TOP-DOWN: find the lowest Y where arm_spread > 0.20m.
    # Above this: arms merge with torso (shoulder region).
    # Below this: arms hang alongside body (forearm/hand region).
    # The transition point IS the armpit.
    armpit_y_val = None
    for s in sorted(sections, key=lambda x: -x["y"]):  # top to bottom
        if s["y"] > 1.45 or s["y"] < 1.10:
            continue
        if not s["has_arms"]:
            continue
        arm_spread = s["total_width"] - s["torso"]["width"]
        if arm_spread > 0.30:
            armpit_y_val = s["y"]
            break  # First Y (scanning down) where arms clearly separate
    
    if armpit_y_val is not None:
        landmarks["armpit_y"] = round(armpit_y_val, 4)
    else:
        # Fallback
        landmarks["armpit_y"] = round(landmarks.get("shoulder_y", 1.4) - 0.20, 4)

    # Shoulder: highest Y with arms
    arm_all = [s for s in sections if s["has_arms"] and s["y"] > 1.1]
    if arm_all:
        landmarks["shoulder_y"] = round(max(s["y"] for s in arm_all), 4)
        ss = best(landmarks["shoulder_y"])
        landmarks["shoulder_torso_width"] = round(ss["torso"]["width"], 4)
        landmarks["shoulder_total_width"] = round(ss["total_width"], 4)

    # Neck: narrowest in Y=1.40-1.65
    m = (ys >= 1.40) & (ys <= 1.65)
    if m.any():
        i = np.where(m)[0][np.argmin(ws[m])]
        landmarks["neck_y"] = round(float(ys[i]), 4)
        landmarks["neck_width"] = round(float(ws[i]), 4)

    # Crotch
    waist_y = landmarks.get("waist_y", 1.0)
    m = (ys >= 0.60) & (ys <= waist_y - 0.1) & (ws < 0.12)
    if m.any():
        landmarks["crotch_y"] = round(float(ys[m].max()), 4)

    for k, v in landmarks.items():
        if isinstance(v, float):
            print(f"    {k}: {v:.4f}")

    # ─── Export ───
    profile = {
        "_comment": "Body profile with torso X-limit ±0.20m, median-smoothed",
        "_mesh_source": mesh_path,
        "_torso_x_limit": TORSO_X_LIMIT,
        "bounds": bounds,
        "landmarks": landmarks,
        "cross_sections": sections,
    }

    out_path = Path(output_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w") as f:
        json.dump(profile, f, indent=2)

    print(f"\n  Exported: {out_path} ({out_path.stat().st_size / 1024:.1f} KB)")

    # ─── Summary ───
    print("\n  ═══ KEY MEASUREMENTS ═══")
    print(f"  {'Region':<12}  {'Y':>6}  {'Width':>7}  {'Depth':>7}  {'Circ':>7}  {'ZF':>6}  {'ZB':>6}")
    for name in ["crotch", "hip", "waist", "chest", "armpit", "shoulder", "neck"]:
        ky = f"{name}_y"
        if ky not in landmarks:
            continue
        s = best(landmarks[ky])
        t = s["torso"]
        print(f"  {name:<12}  {s['y']:6.3f}  {t['width']:7.3f}  {t['depth']:7.3f}  "
              f"{t['circumference']:7.3f}  {t['z_front']:6.3f}  {t['z_back']:6.3f}")

    print("\n  ═══ GARMENT REFERENCE ═══")
    cc = landmarks.get("chest_circumference", 1.0)
    print(f"  Chest circ:        {cc:.3f}m ({cc*100:.0f}cm)")
    print(f"  Panel width (½+ease): {cc/2 + 0.02:.3f}m")
    print(f"  Armhole depth:     {landmarks.get('shoulder_y',1.4) - landmarks.get('armpit_y',1.1):.3f}m")
    print(f"  Tank length:       {landmarks.get('shoulder_y',1.4) - landmarks.get('waist_y',1.0):.3f}m")

    return profile


if __name__ == "__main__":
    analyze_body()
