#!/usr/bin/env python3
"""
Crop Surface Model (Height Map) Generator
Final Clean Version: Hard Boundary Crop to Eliminate Turnaround Loops
"""
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import griddata

FIELD_THETA = math.radians(-5.05)
CELL_SIZE_M = 0.35  # Grid resolution (35cm) for clear row/plot separation

# Fixed field dimensions to exclude turnaround flight zones
FIELD_HALF_L = 77.0   # Trimmed along-row extent (cleanly within plot ends)
FIELD_HALF_W = 23.0   # Trimmed across-row extent (cleanly within top/bottom rows)

def load_points():
    npy_path = '/tmp/field_map.npy'
    if os.path.exists(npy_path):
        pts = np.load(npy_path)
        print(f"[+] Loaded {len(pts):,} points from {npy_path}")
        return pts[:, 0], pts[:, 1], pts[:, 2]
    print("[-] /tmp/field_map.npy not found")
    exit(1)

def rotate_points(x, y, theta):
    cos_t = math.cos(-theta)
    sin_t = math.sin(-theta)
    rx = x * cos_t - y * sin_t
    ry = x * sin_t + y * cos_t
    return rx, ry

def build_height_map(x, y, z):
    # 1. Rotate to align with crop row axis
    rx, ry = rotate_points(x, y, FIELD_THETA)

    # 2. Filter vertical outliers
    valid_z = (z >= -0.5) & (z <= 2.5)
    rx, ry, rz = rx[valid_z], ry[valid_z], z[valid_z]

    # 3. Center using percentiles
    min_x, max_x = np.percentile(rx, 1.0), np.percentile(rx, 99.0)
    min_y, max_y = np.percentile(ry, 1.0), np.percentile(ry, 99.0)
    cx = (min_x + max_x) / 2.0
    cy = (min_y + max_y) / 2.0

    lx = rx - cx
    ly = ry - cy

    # 4. Filter strictly inside the 80-plot boundary (discards turnarounds)
    inside = (np.abs(lx) <= FIELD_HALF_L) & (np.abs(ly) <= FIELD_HALF_W)
    lx_f = lx[inside]
    ly_f = ly[inside]
    rz_f = rz[inside]

    # 5. Create regular grid spanning exact field footprint
    grid_x, grid_y = np.mgrid[-FIELD_HALF_L:FIELD_HALF_L:CELL_SIZE_M, -FIELD_HALF_W:FIELD_HALF_W:CELL_SIZE_M]
    
    # 6. Interpolate
    step = max(1, len(rz_f) // 80000)
    sample_pts = np.column_stack((lx_f[::step], ly_f[::step]))
    sample_vals = rz_f[::step]

    print("[+] Interpolating clean field grid...")
    grid_z = griddata(sample_pts, sample_vals, (grid_x, grid_y), method='linear')

    # 7. Ground normalization: set ground baseline to 0.0m
    valid = grid_z[~np.isnan(grid_z)]
    if len(valid) > 0:
        ground_level = np.percentile(valid, 4)
        grid_z = np.maximum(0.0, grid_z - ground_level)

    return grid_z.T, FIELD_HALF_L, FIELD_HALF_W

def plot_height_map(grid, half_l, half_w, out_path):
    fig, ax = plt.subplots(figsize=(16, 5), dpi=250)

    cmap = plt.cm.viridis.copy()
    cmap.set_bad(color='#ffffff')

    im = ax.imshow(
        grid,
        extent=[-half_l, half_l, -half_w, half_w],
        origin='lower',
        cmap=cmap,
        vmin=0.0,
        vmax=0.65,
        aspect='equal'
    )
    cbar = fig.colorbar(im, ax=ax, shrink=0.75, pad=0.02)
    cbar.set_label('Plant Height (m)', fontsize=11)
    ax.set_xlabel('Along-row distance (m)', fontsize=11)
    ax.set_ylabel('Across-row distance (m)', fontsize=11)
    ax.set_title('LiDAR Crop Surface Model (PhenoRob 80-Plot Sub-Area)', fontsize=13, pad=10)
    
    plt.tight_layout()
    plt.savefig(out_path)
    print(f"[+] Saved clean height map to {out_path}")

if __name__ == "__main__":
    x, y, z = load_points()
    grid, half_l, half_w = build_height_map(x, y, z)
    out_img = os.path.expanduser('~/ros2_sandbox_ws/crop_surface_model_clean.png')
    plot_height_map(grid, half_l, half_w, out_img)
