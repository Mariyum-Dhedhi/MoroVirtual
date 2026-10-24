#!/usr/bin/env python3
"""
field_map_accumulator.py — Ground-Relative Plant Accumulator v4

Key design changes from previous attempts:
  - Voxel map stores ABSOLUTE Z (like the original). No stale rel_z values.
  - Ground estimation uses a per-cell 5-lowest-Z buffer (rejects LiDAR noise).
  - Ground correction is applied at PUBLISH TIME using the LATEST converged
    ground data — ALL points always have consistent, up-to-date rel_z values.
  - Maintains a parallel numpy array alongside the dict to avoid the expensive
    np.array(list(dict.values())) conversion at publish time.
"""
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sensor_msgs.msg import PointCloud2, PointField
from px4_msgs.msg import VehicleLocalPosition, VehicleAttitude
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy,
                        QoSHistoryPolicy, QoSDurabilityPolicy)
import sensor_msgs_py.point_cloud2 as pc2

LIDAR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=50)

PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST, depth=1)

VOXEL_RES = 0.08

# ── Ground grid ────────────────────────────────────────────
GRID_RES  = 0.50
GRID_X0   = -30.0;  GRID_X1 = 200.0
GRID_Y0   = -40.0;  GRID_Y1 =  80.0
GRID_NX   = int((GRID_X1 - GRID_X0) / GRID_RES)   # 460
GRID_NY   = int((GRID_Y1 - GRID_Y0) / GRID_RES)   # 240
BUF_K     = 5       # Keep 5 lowest Z per cell
BUF_IDX   = 2       # Use 3rd-lowest as ground estimate


class FieldMapAccumulator(Node):
    def __init__(self):
        super().__init__('field_map_accumulator')
        self._gx = self._gy = self._gz = 0.0
        self._q = [1.0, 0.0, 0.0, 0.0]
        self._have_pos = self._have_att = False

        # ── Voxel storage (ABSOLUTE Z, like original) ───────
        self._voxel_map = {}              # key→idx into _pts
        self._pts = np.empty((0, 3), dtype=np.float32)  # Nx3 (wx,wy,wz)
        self._pts_n = 0                   # number of valid rows
        self._pts_cap = 0                 # allocated rows

        # ── Ground buffer ───────────────────────────────────
        self._gnd_buf = {}                # (gi,gj) → sorted list of ≤5 lowest Z

        # ── Ground grid (rebuilt at publish time) ───────────
        self._gnd_grid = np.full((GRID_NX, GRID_NY), np.inf, dtype=np.float32)
        self._gnd_dirty = True            # ground changed since last grid build

        self._counter = 0

        self.lidar_sub = self.create_subscription(
            PointCloud2, '/uav/lidar/points', self._scan_cb, LIDAR_QOS)
        self.pos_sub = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1',
            self._pos_cb, PX4_QOS)
        self.att_sub = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude',
            self._att_cb, PX4_QOS)
        self.map_pub = self.create_publisher(
            PointCloud2, '/uav/lidar/field_map', LIDAR_QOS)

        self.timer = self.create_timer(0.5, self._timer_pub)
        self.get_logger().info("====================================================")
        self.get_logger().info(" Ground-Relative Accumulator v4 (publish-time corr.)")
        self.get_logger().info(f" Grid {GRID_NX}x{GRID_NY}@{GRID_RES}m | "
                               f"BufK={BUF_K} idx={BUF_IDX}")
        self.get_logger().info("====================================================")

    # ── Pose callbacks (identical to original) ──────────────
    def _pos_cb(self, msg):
        self._gx = msg.y; self._gy = msg.x; self._gz = -msg.z
        self._have_pos = True

    def _att_cb(self, msg):
        self._q = [float(msg.q[i]) for i in range(4)]
        self._have_att = True

    # ── Ensure _pts has room for 'n' more rows ─────────────
    def _ensure_capacity(self, n):
        needed = self._pts_n + n
        if needed <= self._pts_cap:
            return
        new_cap = max(needed, self._pts_cap * 2, 4096)
        new_arr = np.empty((new_cap, 3), dtype=np.float32)
        if self._pts_n > 0:
            new_arr[:self._pts_n] = self._pts[:self._pts_n]
        self._pts = new_arr
        self._pts_cap = new_cap

    # ── Scan callback ──────────────────────────────────────
    def _scan_cb(self, msg):
        if not (self._have_pos and self._have_att):
            return
        if self._gz < 1.0:
            return

        raw = list(pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                    skip_nans=True))
        if not raw:
            return

        n = len(raw)
        p_bx = np.full(n, -0.12, dtype=np.float64)
        p_by = np.array([float(p[1]) for p in raw], dtype=np.float64)
        p_bz = np.array([float(p[0]) for p in raw], dtype=np.float64)

        w, x, y, z = self._q
        r00 = 1 - 2*(y*y+z*z); r01 = 2*(x*y-w*z); r02 = 2*(x*z+w*y)
        r10 = 2*(x*y+w*z); r11 = 1-2*(x*x+z*z); r12 = 2*(y*z-w*x)
        r20 = 2*(x*z-w*y); r21 = 2*(y*z+w*x); r22 = 1-2*(x*x+y*y)

        ned_x = r00*p_bx + r01*p_by + r02*p_bz
        ned_y = r10*p_bx + r11*p_by + r12*p_bz
        ned_z = r20*p_bx + r21*p_by + r22*p_bz

        wx_a = self._gx + ned_y
        wy_a = self._gy + ned_x
        wz_a = self._gz - ned_z

        vr = VOXEL_RES
        vm = self._voxel_map
        gb = self._gnd_buf
        self._ensure_capacity(n)

        for i in range(n):
            wz_i = float(wz_a[i])
            if wz_i < -2.0 or wz_i > 6.0:
                continue

            wx_i = float(wx_a[i])
            wy_i = float(wy_a[i])

            # ── Ground buffer update ────────────────────────
            gi = max(0, min(int((wx_i - GRID_X0) / GRID_RES), GRID_NX - 1))
            gj = max(0, min(int((wy_i - GRID_Y0) / GRID_RES), GRID_NY - 1))
            gk = (gi, gj)

            buf = gb.get(gk)
            if buf is None:
                gb[gk] = [wz_i]
                self._gnd_dirty = True
            elif len(buf) < BUF_K:
                _insort(buf, wz_i)
                self._gnd_dirty = True
            elif wz_i < buf[-1]:
                _insort(buf, wz_i)
                if len(buf) > BUF_K:
                    buf.pop()
                self._gnd_dirty = True

            # ── Voxel insertion (ABSOLUTE Z, like original) ─
            k = (int(wx_i / vr), int(wy_i / vr), int(wz_i / vr))
            cur_idx = vm.get(k)
            if cur_idx is None:
                idx = self._pts_n
                self._pts_n += 1
                vm[k] = idx
                self._pts[idx] = (wx_i, wy_i, wz_i)
            elif wz_i > self._pts[cur_idx, 2]:
                self._pts[cur_idx] = (wx_i, wy_i, wz_i)

    # ── Publish timer ──────────────────────────────────────
    def _timer_pub(self):
        n = self._pts_n
        if n == 0:
            return

        # Rebuild ground grid from buffer (only when changed, ~13ms)
        if self._gnd_dirty:
            g = self._gnd_grid
            g[:] = np.inf
            for (ci, cj), buf in self._gnd_buf.items():
                if len(buf) > BUF_IDX:
                    g[ci, cj] = buf[BUF_IDX]
                elif buf:
                    g[ci, cj] = buf[len(buf) // 2]
            self._gnd_dirty = False

        # Read absolute coords
        pts = self._pts[:n]          # view, no copy
        wx = pts[:, 0]
        wy = pts[:, 1]
        wz = pts[:, 2]

        # Vectorized ground lookup + correction
        gi = np.clip(((wx - GRID_X0) / GRID_RES).astype(np.int32),
                     0, GRID_NX - 1)
        gj = np.clip(((wy - GRID_Y0) / GRID_RES).astype(np.int32),
                     0, GRID_NY - 1)
        gz = self._gnd_grid[gi, gj]

        finite = np.isfinite(gz)
        if not np.any(finite):
            return
        fallback = float(np.median(gz[finite]))
        gz = np.where(finite, gz, fallback)

        rel_z = np.maximum(0.0, wz - gz).astype(np.float32)

        # Build PointCloud2 directly from numpy (fast, no list conversion)
        out_buf = np.empty((n, 3), dtype=np.float32)
        out_buf[:, 0] = wx
        out_buf[:, 1] = wy
        out_buf[:, 2] = rel_z

        hdr = Header()
        hdr.stamp = self.get_clock().now().to_msg()
        hdr.frame_id = 'map'

        cloud = PointCloud2()
        cloud.header = hdr
        cloud.height = 1
        cloud.width = n
        cloud.fields = [
            PointField(name='x', offset=0,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,
                       datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,
                       datatype=PointField.FLOAT32, count=1),
        ]
        cloud.is_bigendian = False
        cloud.point_step = 12
        cloud.row_step = 12 * n
        cloud.is_dense = True
        cloud.data = out_buf.tobytes()
        self.map_pub.publish(cloud)

        # Periodic save & diagnostics
        self._counter += 1
        if self._counter % 10 == 0:
            np.save('/tmp/field_map.npy', out_buf)
        if self._counter % 40 == 0:
            p95 = float(np.percentile(rel_z, 95)) if n > 0 else 0.0
            n_gnd = len(self._gnd_buf)
            self.get_logger().info(
                f'voxels={n:,}  gnd_cells={n_gnd:,}  '
                f'rel_z_p95={p95:.3f}m')


def _insort(buf, val):
    """Binary-search insert into a sorted list."""
    lo, hi = 0, len(buf)
    while lo < hi:
        mid = (lo + hi) // 2
        if buf[mid] < val:
            lo = mid + 1
        else:
            hi = mid
    buf.insert(lo, val)


def main():
    rclpy.init()
    node = FieldMapAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
