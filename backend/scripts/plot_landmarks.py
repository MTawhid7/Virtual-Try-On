"""
2D Panel Landmark Diagnostic — plots each panel's 2D outline with detected
landmarks (underarm, outer_shoulder, inner_shoulder, neck) and stitch edges
annotated by vertex index.

Usage:
    python -m scripts.plot_landmarks

Saves a PNG to storage/landmarks_debug.png and also displays interactively.
Each subplot = one panel. Landmarks are color-coded dots with vertex-index labels.
Stitch edges are highlighted on the relevant panel as thick coloured lines.
"""

import os
import sys
import numpy as np
import matplotlib
try:
    matplotlib.use("MacOSX")   # interactive on macOS
except Exception:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

backend_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if backend_dir not in sys.path:
    sys.path.insert(0, backend_dir)

from scripts.import_dxf import (
    extract_pieces,
    normalize_piece,
    mirror_piece,
    get_top_landmarks,
    get_sleeve_points,
    compute_stitches,
)

DXF_PATH     = os.path.join(backend_dir, "data/patterns/Polo-Shirt.dxf")
OUT_PATH     = os.path.join(backend_dir, "storage/landmarks_debug.png")
os.makedirs(os.path.dirname(OUT_PATH), exist_ok=True)

# ── Extract + normalise + mirror ──────────────────────────────────────────────
raw = extract_pieces(DXF_PATH, ["Body_Front_1_M", "Body_Back_M", "Sleeves_M"])
for p in raw:
    normalize_piece(p)

pieces: dict = {}
for p in raw:
    if "front" in p.name.lower():
        pieces["front_right"] = p; p.name = "front_right"
        pieces["front_left"]  = mirror_piece(p, "front_left")
    elif "back" in p.name.lower():
        pieces["back_right"] = p; p.name = "back_right"
        pieces["back_left"]  = mirror_piece(p, "back_left")
    elif "sleeve" in p.name.lower():
        pieces["sleeve_right"] = p; p.name = "sleeve_right"
        pieces["sleeve_left"]  = mirror_piece(p, "sleeve_left")

# ── Load stitch definitions (for edge highlighting) ───────────────────────────
stitches = compute_stitches(pieces)

# Build a mapping: panel_name → list of (edge_a_or_b, colour, label)
stitch_edges: dict[str, list] = {k: [] for k in pieces}
STITCH_COLOURS = [
    "#FF4444", "#FF8800", "#FFCC00", "#44FF44",
    "#00CCFF", "#8844FF", "#FF44CC", "#AAAAAA",
    "#FFAAAA", "#AAFFAA", "#AAAAFF", "#FFFF88",
]
for si, s in enumerate(stitches):
    col = STITCH_COLOURS[si % len(STITCH_COLOURS)]
    label = s.get("comment", f"stitch {si}")
    pa, ea = s["panel_a"], s["edge_a"]
    pb, eb = s["panel_b"], s["edge_b"]
    if pa in stitch_edges: stitch_edges[pa].append((ea, col, label))
    if pb in stitch_edges: stitch_edges[pb].append((eb, col, label))

# ── Landmark helper ───────────────────────────────────────────────────────────
PANEL_SIDE = {
    "front_right": "right", "front_left": "left",
    "back_right":  "left",  "back_left":  "right",
}

def get_landmarks(name, verts):
    side = PANEL_SIDE.get(name)
    if side:
        neck, inner_sh, outer_sh, underarm = get_top_landmarks(verts, side)
        return {
            "underarm":       (underarm, "#FF4444"),
            "outer_shoulder": (outer_sh,  "#FF8800"),
            "inner_shoulder": (inner_sh,  "#FFCC00"),
            "neck":           (neck,      "#44CCFF"),
        }
    if "sleeve" in name:
        cL, cR, uL, uR, notch = get_sleeve_points(verts)
        return {
            "cuff_L":    (cL,    "#44FF44"),
            "cuff_R":    (cR,    "#44FF44"),
            "underarm_L":(uL,    "#FF4444"),
            "underarm_R":(uR,    "#FF4444"),
            "notch":     (notch, "#FF8800"),
        }
    return {}

# ── Plot ──────────────────────────────────────────────────────────────────────
panel_names = list(pieces.keys())
ncols = 4
nrows = (len(panel_names) + ncols - 1) // ncols
fig, axes = plt.subplots(nrows, ncols, figsize=(5 * ncols, 6 * nrows))
axes = np.array(axes).flatten()

for ax, name in zip(axes, panel_names):
    verts = pieces[name].vertices_2d
    pts   = np.array(verts)
    xs, ys = pts[:, 0], pts[:, 1]

    # Close polygon for drawing
    closed_x = np.append(xs, xs[0])
    closed_y = np.append(ys, ys[0])

    ax.set_facecolor("#111111")
    ax.plot(closed_x * 100, closed_y * 100, color="#555555", lw=1, zorder=1)
    ax.fill(xs * 100, ys * 100, color="#1a4a5a", alpha=0.5, zorder=0)

    # Stitch edges
    seen_labels = set()
    for edge_idx, col, label in stitch_edges.get(name, []):
        i0, i1 = edge_idx
        n = len(verts)
        # Walk the shorter boundary path between i0 and i1
        d_fwd = (i1 - i0) % n
        d_bwd = (i0 - i1) % n
        if d_fwd <= d_bwd:
            path = [pts[(i0 + k) % n] for k in range(d_fwd + 1)]
        else:
            path = [pts[(i0 - k) % n] for k in range(d_bwd + 1)]
        px = [p[0] * 100 for p in path]
        py = [p[1] * 100 for p in path]
        lbl = label if label not in seen_labels else None
        seen_labels.add(label)
        ax.plot(px, py, color=col, lw=3, alpha=0.85, label=lbl, zorder=3)

    # Vertex index labels (every Nth to avoid clutter)
    n = len(verts)
    step = max(1, n // 20)
    for i in range(0, n, step):
        ax.text(xs[i] * 100, ys[i] * 100, str(i),
                fontsize=5, color="#888888", ha="center", va="center", zorder=4)

    # Landmark dots + labels
    lm = get_landmarks(name, verts)
    for lm_name, (idx, col) in lm.items():
        x, y = pts[idx, 0] * 100, pts[idx, 1] * 100
        ax.scatter([x], [y], s=60, color=col, zorder=5, edgecolors="white", linewidths=0.5)
        ax.text(x + 0.5, y + 0.5, f"v{idx}\n{lm_name}",
                fontsize=6, color=col, zorder=6,
                bbox=dict(boxstyle="round,pad=0.1", fc="#111111", alpha=0.7))

    ax.set_title(name, color="white", fontsize=9)
    ax.set_xlabel("X (cm)", color="#aaaaaa", fontsize=7)
    ax.set_ylabel("Y (cm)", color="#aaaaaa", fontsize=7)
    ax.tick_params(colors="#666666", labelsize=6)
    ax.set_aspect("equal")
    for spine in ax.spines.values():
        spine.set_edgecolor("#333333")

    # Stitch legend
    handles = [mpatches.Patch(color=c, label=l)
               for e, c, l in stitch_edges.get(name, []) if l not in (None,)]
    if handles:
        ax.legend(handles=handles, fontsize=5, loc="lower right",
                  facecolor="#1a1a1a", edgecolor="#444444", labelcolor="white")

# Hide unused axes
for ax in axes[len(panel_names):]:
    ax.set_visible(False)

fig.patch.set_facecolor("#0a0a0a")
fig.suptitle("DXF Panel Landmarks & Stitch Edges", color="white", fontsize=13, y=1.01)
plt.tight_layout()
plt.savefig(OUT_PATH, dpi=150, bbox_inches="tight", facecolor="#0a0a0a")
print(f"  Saved: {OUT_PATH}")
plt.show()
