#!/usr/bin/env python3
"""
plot_coverage_tight_and_chessboard.py
Dual-Pass Chessboard (Cross-Hatch) Trajectory over 80 Active Crop Plots.
Generates Phase 1 (Along-Row) + Phase 2 (Across-Row) Waypoints.
"""

import math
import numpy as np
import matplotlib.pyplot as plt
import shapefile

from get_boundaries import (
    get_field_boundaries_from_shapefile,
    latlon_to_local_xy,
    SHP_PATH,
)

NUM_PASSES_ALONG = 8
NUM_PASSES_ACROSS = 18

def load_active_80_plots():
    _, gps_bounds, _ = get_field_boundaries_from_shapefile(SHP_PATH)
    lat_origin, lon_origin = gps_bounds[2], gps_bounds[0]

    sf = shapefile.Reader(SHP_PATH)
    all_polys = []
    for s in sf.shapes():
        poly = [latlon_to_local_xy(lat, lon, lat_origin, lon_origin) for lon, lat in s.points]
        all_polys.append(poly)

    active_80_plots = [
        poly for poly in all_polys
        if (sum(p[0] for p in poly) / len(poly)) <= 168.0
    ]
    return active_80_plots

def compute_field_geometry(active_80_plots):
    all_pts = []
    for poly in active_80_plots:
        all_pts.extend(poly)
    all_pts = np.array(all_pts)

    m, _ = np.polyfit(all_pts[:, 0], all_pts[:, 1], 1)
    theta = math.atan(m)

    c, s = math.cos(-theta), math.sin(-theta)
    rot_matrix = np.array([[c, -s], [s, c]])

    rotated_pts = np.dot(all_pts, rot_matrix.T)
    min_rx, max_rx = rotated_pts[:, 0].min(), rotated_pts[:, 0].max()
    min_ry, max_ry = rotated_pts[:, 1].min(), rotated_pts[:, 1].max()

    center_rx = (min_rx + max_rx) / 2.0
    center_ry = (min_ry + max_ry) / 2.0

    c_inv, s_inv = math.cos(theta), math.sin(theta)
    cx = center_rx * c_inv - center_ry * s_inv
    cy = center_rx * s_inv + center_ry * c_inv

    half_l = (max_rx - min_rx) / 2.0 + 1.0
    half_w = (max_ry - min_ry) / 2.0

    return cx, cy, theta, half_l, half_w

def generate_along_row_waypoints(cx, cy, theta, half_l, half_w, num_lanes):
    y_lanes = np.linspace(-half_w + 1.8, half_w - 1.8, num_lanes)
    local_wps = []
    direction = 1
    for y in y_lanes:
        if direction == 1:
            local_wps.append((-half_l, y))
            local_wps.append((half_l, y))
        else:
            local_wps.append((half_l, y))
            local_wps.append((-half_l, y))
        direction *= -1

    c, s = math.cos(theta), math.sin(theta)
    world_wps = [(cx + (lx * c - ly * s), cy + (lx * s + ly * c)) for lx, ly in local_wps]
    return world_wps

def generate_across_row_waypoints(cx, cy, theta, half_l, half_w, num_lanes):
    x_lanes = np.linspace(half_l - 2.0, -half_l + 2.0, num_lanes)
    local_wps = []
    direction = 1
    for x in x_lanes:
        if direction == 1:
            local_wps.append((x, half_w))
            local_wps.append((x, -half_w))
        else:
            local_wps.append((x, -half_w))
            local_wps.append((x, half_w))
        direction *= -1

    c, s = math.cos(theta), math.sin(theta)
    world_wps = [(cx + (lx * c - ly * s), cy + (lx * s + ly * c)) for lx, ly in local_wps]
    return world_wps

def generate_chessboard_waypoints(cx, cy, theta, half_l, half_w):
    wps_along = generate_along_row_waypoints(cx, cy, theta, half_l, half_w, NUM_PASSES_ALONG)
    wps_across = generate_across_row_waypoints(cx, cy, theta, half_l, half_w, NUM_PASSES_ACROSS)
    return wps_along + wps_across

def draw_field(ax, active_80_plots, cx, cy, theta, half_l, half_w):
    for idx, poly in enumerate(active_80_plots):
        xs = [p[0] for p in poly]
        ys = [p[1] for p in poly]
        lbl = '80 Active Trial Plots' if idx == 0 else ""
        ax.fill(xs, ys, facecolor="#cdeac0", edgecolor="#386641", linewidth=0.8, alpha=0.9, label=lbl)

    c, s = math.cos(theta), math.sin(theta)
    corners = [
        (-half_l, -half_w),
        (half_l, -half_w),
        (half_l, half_w),
        (-half_l, half_w),
        (-half_l, -half_w)
    ]
    bx = [cx + (lx * c - ly * s) for lx, ly in corners]
    by = [cy + (lx * s + ly * c) for lx, ly in corners]
    ax.plot(bx, by, '--', color='#333333', linewidth=1.2, label='Survey Bounding Box')

def main():
    plots = load_active_80_plots()
    cx, cy, theta, half_l, half_w = compute_field_geometry(plots)

    wps_along = generate_along_row_waypoints(cx, cy, theta, half_l, half_w, NUM_PASSES_ALONG)
    wps_across = generate_across_row_waypoints(cx, cy, theta, half_l, half_w, NUM_PASSES_ACROSS)

    fig, ax = plt.subplots(figsize=(16, 7), dpi=300)
    draw_field(ax, plots, cx, cy, theta, half_l, half_w)

    ax.plot(0, 0, '^', color='#0077b6', markersize=10, label='Home (0,0)')

    # Phase 1: Along-row trajectory
    along_x = [0.0] + [wp[0] for wp in wps_along]
    along_y = [0.0] + [wp[1] for wp in wps_along]
    ax.plot(along_x, along_y, '-', color='#b7094c', linewidth=1.4,
            label=f'Phase 1: Along-Row Passes ({NUM_PASSES_ALONG} Passes)')

    # Phase 2: Across-row trajectory
    across_x = [wps_along[-1][0]] + [wp[0] for wp in wps_across] + [0.0]
    across_y = [wps_along[-1][1]] + [wp[1] for wp in wps_across] + [0.0]
    ax.plot(across_x, across_y, '-', color='#0077b6', linewidth=1.2, alpha=0.85,
            label=f'Phase 2: Across-Row Cross-Hatch ({NUM_PASSES_ACROSS} Passes)')

    ax.plot(wps_along[0][0], wps_along[0][1], 'o', color='#b7094c', markersize=5)
    ax.text(wps_along[0][0] + 1.5, wps_along[0][1] - 2, 'Start Along', fontsize=8, color='#b7094c', weight='bold')

    ax.plot(wps_across[0][0], wps_across[0][1], 'o', color='#0077b6', markersize=5)
    ax.text(wps_across[0][0] - 14, wps_across[0][1] + 2, 'Start Across', fontsize=8, color='#0077b6', weight='bold')

    total_wps = len(wps_along) + len(wps_across)
    ax.set_xlabel('Local East X (metres from field SW corner)', fontsize=11)
    ax.set_ylabel('Local North Y (metres from field SW corner)', fontsize=11)
    ax.set_title(f'Dual-Pass Chessboard UAV Survey Trajectory (80 Active Plots)\n'
                 f'Total Passes: {NUM_PASSES_ALONG + NUM_PASSES_ACROSS} ({total_wps} WPs) | Altitude: 6.0 m | Angle: {math.degrees(theta):.2f}°',
                 fontsize=12, fontweight='bold')
    ax.grid(True, linestyle=':', alpha=0.6)
    ax.legend(loc='upper right', framealpha=0.95)
    ax.set_aspect('equal')
    plt.tight_layout()
    out = '/home/abdullah/ros2_sandbox_ws/trajectory_chessboard_80_plots.png'
    plt.savefig(out)
    print(f"[+] Saved dual-pass chessboard orthophoto to {out}")

if __name__ == '__main__':
    main()
