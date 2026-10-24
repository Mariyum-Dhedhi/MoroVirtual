#!/usr/bin/env python3
"""
field_coverage_flight_tight.py — Dual-Pass Chessboard Flight Node
Executes full Along-Row + Across-Row cross-hatch scan in Gazebo.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)

from plot_coverage_tight_and_chessboard import (
    load_active_80_plots,
    compute_field_geometry,
    generate_chessboard_waypoints,
    NUM_PASSES_ALONG,
    NUM_PASSES_ACROSS,
)

PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
)

SCAN_SPEED      =  2.5    # m/s
CRUISE_ALT      = -6.0    # NED metres (-6.0m = 6m altitude)
WAYPOINT_RADIUS =  1.5    # metres
PREARM_CYCLES   =  15

GAZEBO_Y_OFFSET = -4.0

plots = load_active_80_plots()
CX, CY, THETA, HALF_L, HALF_W = compute_field_geometry(plots)
raw_wps = generate_chessboard_waypoints(CX, CY, THETA, HALF_L, HALF_W)

WAYPOINTS = [(wp[0], wp[1] + GAZEBO_Y_OFFSET, CRUISE_ALT) for wp in raw_wps]


class FieldCoverageFlight(Node):
    def __init__(self):
        super().__init__('field_coverage_flight')

        self.get_logger().info("==========================================")
        self.get_logger().info(f" Chessboard Flight Initialized: {len(WAYPOINTS)} Total Waypoints")
        self.get_logger().info(f" Along Passes : {NUM_PASSES_ALONG} | Across Passes: {NUM_PASSES_ACROSS}")
        self.get_logger().info(f" Start WP 1   : ({WAYPOINTS[0][0]:.2f}, {WAYPOINTS[0][1]:.2f})")
        self.get_logger().info(f" Cruise Alt   : {abs(CRUISE_ALT)} m | Speed: {SCAN_SPEED} m/s")
        self.get_logger().info("==========================================")

        self._pub_mode = self.create_publisher(OffboardControlMode, "/fmu/in/offboard_control_mode", PX4_QOS)
        self._pub_tsp  = self.create_publisher(TrajectorySetpoint,  "/fmu/in/trajectory_setpoint",   PX4_QOS)
        self._pub_cmd  = self.create_publisher(VehicleCommand,       "/fmu/in/vehicle_command",       PX4_QOS)

        self.create_subscription(VehicleLocalPosition, "/fmu/out/vehicle_local_position_v1", self._pos_cb, PX4_QOS)
        self.create_subscription(VehicleStatus,        "/fmu/out/vehicle_status_v4",         self._status_cb, PX4_QOS)

        self._state = "INIT"
        self._pos = None
        self._status = None
        self._wp_idx = 0
        self._stream_cnt = 0
        self._home_x = 0.0
        self._home_y = 0.0

        self._timer = self.create_timer(0.05, self._control_loop)

    def _pos_cb(self, msg):
        self._pos = msg

    def _status_cb(self, msg):
        self._status = msg

    def _control_loop(self):
        if not self._pos:
            return

        cur_gx = self._pos.y
        cur_gy = self._pos.x
        cur_gz = -self._pos.z

        self._publish_offboard_mode()

        if self._state == "INIT":
            self._send_setpoint(cur_gx, cur_gy, CRUISE_ALT)
            self._stream_cnt += 1
            if self._stream_cnt >= PREARM_CYCLES:
                self._home_x = cur_gx
                self._home_y = cur_gy
                self.get_logger().info(f"Arming at Home ({self._home_x:.1f}, {self._home_y:.1f})...")
                self._arm()
                self._set_offboard()
                self._state = "TAKEOFF"

        elif self._state == "TAKEOFF":
            self._send_setpoint(self._home_x, self._home_y, CRUISE_ALT)
            if abs(cur_gz - abs(CRUISE_ALT)) < 0.5:
                self.get_logger().info("Altitude reached. Starting Dual-Pass Chessboard Survey...")
                self._state = "SCANNING"

        elif self._state == "SCANNING":
            tx, ty, tz = WAYPOINTS[self._wp_idx]
            dx = tx - cur_gx
            dy = ty - cur_gy
            dist_xy = math.sqrt(dx*dx + dy*dy)

            vx = (dx / (dist_xy + 1e-6)) * SCAN_SPEED
            vy = (dy / (dist_xy + 1e-6)) * SCAN_SPEED

            self._send_setpoint(tx, ty, tz, vx=vx, vy=vy)

            if dist_xy < WAYPOINT_RADIUS:
                phase = "Along-Row" if self._wp_idx < (NUM_PASSES_ALONG * 2) else "Across-Row"
                self.get_logger().info(f"[{phase}] WP {self._wp_idx+1}/{len(WAYPOINTS)}: ({tx:.1f}, {ty:.1f})")
                self._wp_idx += 1
                if self._wp_idx >= len(WAYPOINTS):
                    self.get_logger().info("Chessboard scan completed. Returning Home...")
                    self._state = "RETURN"

        elif self._state == "RETURN":
            dx = self._home_x - cur_gx
            dy = self._home_y - cur_gy
            dist_xy = math.sqrt(dx*dx + dy*dy)
            vx = (dx / (dist_xy + 1e-6)) * SCAN_SPEED
            vy = (dy / (dist_xy + 1e-6)) * SCAN_SPEED
            self._send_setpoint(self._home_x, self._home_y, CRUISE_ALT, vx=vx, vy=vy)

            if dist_xy < WAYPOINT_RADIUS:
                self.get_logger().info("Arrived above Home. Controlled descent starting...")
                self._state = "LANDING"

        elif self._state == "LANDING":
            self._send_setpoint(self._home_x, self._home_y, 0.0, vz=0.5)

            if abs(self._pos.z) < 0.3 or cur_gz < 0.3:
                self.get_logger().info("Touchdown reached. Disarming vehicle.")
                self._disarm()
                self._state = "COMPLETED"

        elif self._state == "COMPLETED":
            pass

    def _publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_mode.publish(msg)

    def _send_setpoint(self, gx, gy, gz, vx=0.0, vy=0.0, vz=0.0):
        msg = TrajectorySetpoint()
        msg.position = [gy, gx, gz]
        msg.velocity = [vy, vx, vz]
        if abs(vx) > 0.1 or abs(vy) > 0.1:
            msg.yaw = math.atan2(vx, vy)
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_tsp.publish(msg)

    def _arm(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_cmd.publish(cmd)

    def _disarm(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 0.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_cmd.publish(cmd)

    def _set_offboard(self):
        cmd = VehicleCommand()
        cmd.command = VehicleCommand.VEHICLE_CMD_DO_SET_MODE
        cmd.param1 = 1.0
        cmd.param2 = 6.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        self._pub_cmd.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = FieldCoverageFlight()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
