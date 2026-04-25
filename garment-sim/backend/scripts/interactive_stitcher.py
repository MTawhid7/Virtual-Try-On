import json
import argparse
import numpy as np
import matplotlib.pyplot as plt
import os

def load_pattern(path):
    with open(path, 'r') as f:
        return json.load(f)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pattern', default='data/patterns/tshirt.json')
    args = parser.parse_args()

    pattern = load_pattern(args.pattern)
    panels = pattern['panels']

    # Layout panels side by side
    offsets = []
    current_x = 0
    all_points_2d = []
    point_meta = [] # (panel_idx, vertex_idx)

    for p_idx, p in enumerate(panels):
        verts = np.array(p['vertices_2d'])
        min_x = verts[:, 0].min()
        max_x = verts[:, 0].max()
        
        offset_x = current_x - min_x
        
        for v_idx, v in enumerate(verts):
            all_points_2d.append([v[0] + offset_x, v[1]])
            point_meta.append((p_idx, v_idx))
            
        offsets.append(offset_x)
        current_x += (max_x - min_x) + 0.1 # 10cm padding

    all_points_2d = np.array(all_points_2d)

    fig, ax = plt.subplots(figsize=(14, 9))
    fig.canvas.manager.set_window_title("Garment Stitching Pro - Topo Editor")
    ax.set_aspect('equal')
    ax.axis('off')

    # Draw panels
    for p_idx, p in enumerate(panels):
        verts = np.array(p['vertices_2d'])
        verts_offset = verts.copy()
        verts_offset[:, 0] += offsets[p_idx]
        
        poly = plt.Polygon(verts_offset, fill=True, facecolor='#f0f0f0', alpha=0.9, edgecolor='#333333', linewidth=1.5)
        ax.add_patch(poly)
        
        # High-res points tiny
        # ax.plot(verts_offset[:, 0], verts_offset[:, 1], 'k.', markersize=2, alpha=0.3)
        
        # Highlight CORNER points large (original vertices)
        ax.plot(verts_offset[:, 0], verts_offset[:, 1], 'o', color='#ffcc00', markersize=8, markeredgecolor='black', alpha=0.8)
        
        # Labels removed for clarity as requested
        # for v_idx, v in enumerate(verts_offset):
        #     ax.text(v[0], v[1], str(v_idx), fontsize=8, color='blue', alpha=0.7, fontweight='bold')
            
        ax.text(verts_offset[:, 0].mean(), verts_offset[:, 1].max() + 0.05, p['id'].upper(), 
                ha='center', fontweight='bold', fontsize=12, color='#2c3e50')

    pt_scatter = ax.scatter(all_points_2d[:, 0], all_points_2d[:, 1], s=40, c='none', picker=True, pickradius=15)

    clicks = []       
    saved_stitches = [] 
    lines = []        

    instruction_text = ax.text(0.5, 0.01, "Step 1: Click the START of Edge A (Red)", 
                               transform=ax.transAxes, ha='center', fontsize=14, 
                               fontweight='bold', color='firebrick', bbox=dict(facecolor='white', alpha=0.8, pad=5))

    def update_plot():
        for l in lines:
            l.remove()
        lines.clear()

        # Update instructions
        steps = [
            "Step 1: Click the START of Edge A (Red)",
            "Step 2: Click the END of Edge A (Red)",
            "Step 3: Click the START of matching Edge B (Blue)",
            "Step 4: Click the END of matching Edge B (Blue)"
        ]
        instruction_text.set_text(steps[len(clicks)] if len(clicks) < 4 else steps[0])
        instruction_text.set_color('firebrick' if len(clicks) < 2 else 'royalblue')

        # Draw already saved stitches
        for idx, st in enumerate(saved_stitches):
            pa = next(i for i, p in enumerate(panels) if p['id'] == st['panel_a'])
            pb = next(i for i, p in enumerate(panels) if p['id'] == st['panel_b'])
            
            va1, va2 = st['edge_a']
            vb1, vb2 = st['edge_b']
            
            p1 = np.array(panels[pa]['vertices_2d'][va1]) + [offsets[pa], 0]
            p2 = np.array(panels[pa]['vertices_2d'][va2]) + [offsets[pa], 0]
            p3 = np.array(panels[pb]['vertices_2d'][vb1]) + [offsets[pb], 0]
            p4 = np.array(panels[pb]['vertices_2d'][vb2]) + [offsets[pb], 0]
            
            l1, = ax.plot([p1[0], p2[0]], [p1[1], p2[1]], color='red', linewidth=4, alpha=0.8)
            l2, = ax.plot([p3[0], p4[0]], [p3[1], p4[1]], color='blue', linewidth=4, alpha=0.8)
            
            # Connection threads
            l3, = ax.plot([p1[0], p3[0]], [p1[1], p3[1]], color='green', linestyle=':', linewidth=1)
            l4, = ax.plot([p2[0], p4[0]], [p2[1], p4[1]], color='green', linestyle=':', linewidth=1)
            
            midx = (p1[0] + p3[0]) / 2
            midy = (p1[1] + p3[1]) / 2
            t = ax.text(midx, midy, f"#{idx+1}", color='green', fontsize=10, 
                        fontweight='bold', ha='center', bbox=dict(facecolor='white', alpha=0.9, boxstyle='round,pad=0.2'))
            
            lines.extend([l1, l2, l3, l4, t])

        # Draw current clicks
        for i, c_idx in enumerate(clicks):
            color = 'red' if i < 2 else 'blue'
            p = all_points_2d[c_idx]
            l1 = ax.scatter(p[0], p[1], s=120, edgecolors='black', facecolors=color, zorder=10)
            lines.append(l1)
            if i == 1: # Red line
                p0 = all_points_2d[clicks[0]]
                l2, = ax.plot([p0[0], p[0]], [p0[1], p[1]], color='red', linewidth=4)
                lines.append(l2)
            if i == 3: # Blue line (briefly before clearing)
                p2 = all_points_2d[clicks[2]]
                l2, = ax.plot([p2[0], p[0]], [p2[1], p[1]], color='blue', linewidth=4)
                lines.append(l2)
            
        fig.canvas.draw_idle()

    def on_click(event):
        if event.inaxes != ax: return
        if event.button != 1: return
        
        # Find nearest point manually
        if event.xdata is None or event.ydata is None: return
        dists = np.sum((all_points_2d - [event.xdata, event.ydata])**2, axis=1)
        nearest_idx = np.argmin(dists)
        
        if dists[nearest_idx] > 0.05:  # generous threshold for corners
            return
            
        clicks.append(nearest_idx)
        print(f"Selecting: Panel={panels[point_meta[nearest_idx][0]]['id']} | Vertex={point_meta[nearest_idx][1]}")
        
        if len(clicks) == 4:
            p_a1, v_a1 = point_meta[clicks[0]]
            p_a2, v_a2 = point_meta[clicks[1]]
            p_b1, v_b1 = point_meta[clicks[2]]
            p_b2, v_b2 = point_meta[clicks[3]]
            
            if p_a1 != p_a2:
                print("⚠️ Error: Source Edge must be on ONE panel.")
                clicks.clear()
            elif p_b1 != p_b2:
                print("⚠️ Error: Target Edge must be on ONE panel.")
                clicks.clear()
            else:
                stitch = {
                    "comment": f"Sew {panels[p_a1]['id']} edge {v_a1}-{v_a2} to {panels[p_b1]['id']} edge {v_b1}-{v_b2}",
                    "panel_a": panels[p_a1]['id'],
                    "edge_a": [v_a1, v_a2],
                    "panel_b": panels[p_b1]['id'],
                    "edge_b": [v_b1, v_b2]
                }
                saved_stitches.append(stitch)
                print(f"✅ Created stitch: {stitch['comment']}")
                clicks.clear()
        
        update_plot()

    def on_key(event):
        # Force lower case for reliability
        key = (event.key or "").lower()
        if key in ['z', 'u', 'backspace']:
            if len(clicks) > 0:
                clicks.pop()
                print("Undid click.")
            elif len(saved_stitches) > 0:
                saved_stitches.pop()
                print("Undid last stitch.")
            update_plot()
            
        elif key == 's':
            if len(saved_stitches) == 0:
                print("No stitches to save.")
                return
            
            pattern['stitches'] = saved_stitches
            with open(args.pattern, 'w') as f:
                json.dump(pattern, f, indent=2)
            print(f"\n💾  SAVED {len(saved_stitches)} STITCHES TO {args.pattern}")
            print("You can close this window now and run the simulation.")

    fig.canvas.mpl_connect('button_press_event', on_click)
    fig.canvas.mpl_connect('key_press_event', on_key)

    print("\n" + "═"*50)
    print("  GARMENT STITCHING PRO - READY")
    print("═"*50)
    print(" 1. Click two corners of PANEL A (Source Edge)")
    print(" 2. Click two corners of PANEL B (Target Edge)")
    print(" 3. Repeat for all seams (Shoulders, Sides, Armholes)")
    print(" 4. Press 'S' to save.")
    print("\n HOTKEYS:")
    print("   [Z] or [U] : Undo last action")
    print("   [S]        : Save everything")
    print("═"*50 + "\n")
    
    update_plot()
    plt.show()

if __name__ == '__main__':
    main()
