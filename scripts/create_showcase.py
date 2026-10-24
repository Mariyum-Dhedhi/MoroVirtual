#!/usr/bin/env python3
"""Render the saved planned route; this is not measured robot motion."""
import csv
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'docs/media'

def main():
    OUT.mkdir(parents=True, exist_ok=True)
    with (ROOT / 'src/Viz_shapefiles/outputs/waypoints.csv').open() as f:
        rows = sorted(csv.DictReader(f), key=lambda r: int(r['index']))
    points = np.array([(float(r['x']), float(r['y'])) for r in rows])
    lengths = np.linalg.norm(np.diff(points, axis=0), axis=1)
    distance = np.r_[0, np.cumsum(lengths)]
    samples = np.linspace(0, distance[-1], 160)
    route = np.column_stack([np.interp(samples, distance, points[:, i]) for i in (0, 1)])
    plt.style.use('dark_background')
    fig, ax = plt.subplots(figsize=(10, 5.6), dpi=100)
    fig.patch.set_facecolor('#101923')
    ax.set_facecolor('#101923')
    ax.plot(*points.T, color='#395267', linewidth=1, label='Saved waypoint route')
    ax.scatter(*points[[0, -1]].T, c=['#69e0ad', '#ffc66b'], s=45, zorder=4)
    trail, = ax.plot([], [], color='#69e0ad', lw=2.2)
    marker, = ax.plot([], [], 'o', color='#ffffff', ms=6)
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('Local x (m)')
    ax.set_ylabel('Local y (m)')
    ax.grid(alpha=.13)
    ax.set_title('MoroVirtual | Agricultural field navigation', loc='left', pad=18)
    fig.text(.5, .02, 'PLANNED ROUTE PREVIEW • interpolated CSV waypoints • not a simulation recording', ha='center', fontsize=9, color='#a9bdcd')
    ax.legend(loc='upper right', fontsize=8)
    fig.tight_layout(rect=(0,.04,1,1))
    fig.savefig(OUT / 'planned-route.png', dpi=160)
    def update(i):
        trail.set_data(route[:i+1,0],route[:i+1,1])
        marker.set_data([route[i,0]],[route[i,1]])
        return trail, marker
    animation = FuncAnimation(fig, update, frames=len(route), interval=75, blit=True)
    animation.save(OUT / 'planned-route.gif', writer=PillowWriter(fps=12))
    try:
        import cv2
        writer = cv2.VideoWriter(str(OUT / 'planned-route.mp4'), cv2.VideoWriter_fourcc(*'mp4v'), 12, (1000,560))
        if not writer.isOpened():
            raise RuntimeError('MP4 writer unavailable')
        for i in range(len(route)):
            update(i)
            fig.canvas.draw()
            rgb = np.asarray(fig.canvas.buffer_rgba())[:,:,:3]
            writer.write(cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
        writer.release()
    except ImportError:
        print('OpenCV unavailable; GIF and PNG created. Install opencv-python for MP4.')
    print(f'{len(rows)} waypoints; planned polyline length {distance[-1]:.1f} m. Media: {OUT}')
    plt.close(fig)

if __name__ == '__main__':
    main()
