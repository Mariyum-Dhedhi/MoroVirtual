#!/usr/bin/env python3
"""
Coverage/completeness map generator.
Uses the same real flight data as the height map, but instead of showing
height, shows how many LiDAR points landed in each grid cell -- direct,
visual proof of autonomous full-field coverage.
"""
import numpy as np
import matplotlib.pyplot as plt
import math

FIELD_CENTER_X = 87.485
FIELD_CENTER_Y = 29.160
FIELD_LENGTH   = 159.00
FIELD_WIDTH    = 33.00
FIELD_THETA    = math.radians(-5.447)
MARGIN_M = 2.0
TURN_TRIM_M = 6.0
CELL_SIZE_M = 1.0

def load_points(npy_path):
    d = np.load(npy_path)
    return d[:,0], d[:,1], d[:,2]

def world_to_field_frame(x, y):
    dx = x - FIELD_CENTER_X
    dy = y - FIELD_CENTER_Y
    cos_t = math.cos(-FIELD_THETA)
    sin_t = math.sin(-FIELD_THETA)
    local_x = dx * cos_t - dy * sin_t
    local_y = dx * sin_t + dy * cos_t
    return local_x, local_y

def build_coverage_map(x, y):
    local_x, local_y = world_to_field_frame(x, y)

    half_l = FIELD_LENGTH / 2.0 + MARGIN_M - TURN_TRIM_M
    half_w = FIELD_WIDTH / 2.0 + MARGIN_M

    in_field = (np.abs(local_x) <= half_l) & (np.abs(local_y) <= half_w)
    lx, ly = local_x[in_field], local_y[in_field]

    n_cols = int(np.ceil(2 * half_l / CELL_SIZE_M))
    n_rows = int(np.ceil(2 * half_w / CELL_SIZE_M))

    grid = np.zeros((n_rows, n_cols))

    col_idx = ((lx + half_l) / CELL_SIZE_M).astype(int)
    row_idx = ((ly + half_w) / CELL_SIZE_M).astype(int)
    col_idx = np.clip(col_idx, 0, n_cols - 1)
    row_idx = np.clip(row_idx, 0, n_rows - 1)

    for r, c in zip(row_idx, col_idx):
        grid[r, c] += 1

    total_cells = n_rows * n_cols
    covered_cells = np.count_nonzero(grid)
    coverage_pct = 100.0 * covered_cells / total_cells

    print(f"Total field cells: {total_cells}")
    print(f"Covered cells (>=1 scan): {covered_cells}")
    print(f"Coverage: {coverage_pct:.1f}%")

    return grid, half_l, half_w, coverage_pct

def plot_coverage_map(grid, half_l, half_w, coverage_pct, out_path):
    fig, ax = plt.subplots(figsize=(14, 4))
    # log scale for scan-count so a few very-high-count cells (rows the
    # drone crossed slowly / multiple passes) don't wash out lightly-scanned areas
    display_grid = np.where(grid > 0, np.log1p(grid), np.nan)
    im = ax.imshow(
        display_grid,
        extent=[-half_l, half_l, -half_w, half_w],
        origin='lower',
        cmap='plasma',
        aspect='equal'
    )
    cbar = fig.colorbar(im, ax=ax, shrink=0.8)
    cbar.set_label('Scan density (log scale)')
    ax.set_xlabel('Along-row distance (m)')
    ax.set_ylabel('Across-row distance (m)')
    ax.set_title(f'Autonomous Field Coverage Map\nReal PhenoRob field, 80-plot sub-area  |  Coverage: {coverage_pct:.1f}%')
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    print(f"Saved coverage map to {out_path}")

if __name__ == "__main__":
    x, y, z = load_points('/tmp/field_map.npy')
    grid, half_l, half_w, coverage_pct = build_coverage_map(x, y)
    plot_coverage_map(grid, half_l, half_w, coverage_pct, '/tmp/coverage_map.png')
