#!/usr/bin/env python3
"""Waypoint follower — pure proportional control, no Nav2/costmap.

Per waypoint: rotate in place to face it, then drive straight at it with
continuous heading correction, then advance once within GOAL_TOLERANCE.
"""

import csv
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PointStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf_transformations import euler_from_quaternion


# Late subscribers (e.g. accuracy_logger starting after the follower) still
# need the last-published target, not just future ones.
CURRENT_TARGET_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

HEADING_DEADBAND     = 0.05  # rad
GOAL_TOLERANCE       = 0.3   # m
LINEAR_SPEED         = 1.0   # m/s
ANGULAR_GAIN         = 1.5
MAX_ANGULAR_SPEED    = 1.0   # rad/s
CONTROL_PERIOD       = 0.2   # s (5 Hz — matches /odometry/filtered rate)
INDEX_PUBLISH_PERIOD = 1.0   # s


def _normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def _load_waypoints(path):
    rows = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            rows.append({
                "index": int(row["index"]),
                "x": float(row["x"]),
                "y": float(row["y"]),
                "point_type": row["point_type"],
            })
    rows.sort(key=lambda r: r["index"])
    return rows


class WaypointFollower(Node):

    def __init__(self):
        super().__init__("waypoint_follower")

        self.declare_parameter("waypoints_file", "")

        waypoints_file = self.get_parameter("waypoints_file").value
        if not waypoints_file:
            raise ValueError("waypoints_file parameter must be set to a CSV path")

        self._waypoints = _load_waypoints(waypoints_file)
        self.get_logger().info(
            f"Loaded {len(self._waypoints)} waypoints from {waypoints_file}")
        if self._waypoints:
            wp0 = self._waypoints[0]
            self.get_logger().info(f"First waypoint: ({wp0['x']:.4f}, {wp0['y']:.4f})")

        self._cmd_pub   = self.create_publisher(Twist, "/cmd_vel", 10)
        self._index_pub = self.create_publisher(
            Int32, "/waypoint_follower/current_index", 10)
        self._target_pub = self.create_publisher(
            PointStamped, "/current_target", CURRENT_TARGET_QOS)
        self._reached_pub = self.create_publisher(Int32, "/waypoint_reached", 10)
        self.create_subscription(
            Odometry, "/odometry/filtered", self._odom_cb, 10)

        self._x = self._y = self._yaw = 0.0
        self._odom_received = False
        self._current_idx   = 0
        self._last_published_target_idx = None
        self._rotating       = True
        self._done = not self._waypoints
        if self._done:
            self.get_logger().warn("No waypoints loaded; nothing to do")

        self.create_timer(CONTROL_PERIOD, self._control_cb)
        self.create_timer(INDEX_PUBLISH_PERIOD, self._publish_index_cb)

    def _odom_cb(self, msg: Odometry):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self._yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._odom_received = True

    def _publish_index_cb(self):
        self._index_pub.publish(Int32(data=self._current_idx))

    def _publish_current_target_if_changed(self, target_x, target_y):
        if self._current_idx == self._last_published_target_idx:
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        # PointStamped has no index field; the target waypoint index is
        # carried in frame_id (e.g. "wp_3") rather than pulling in a
        # custom message type for this one purpose.
        msg.header.frame_id = f"wp_{self._current_idx}"
        msg.point.x = target_x
        msg.point.y = target_y
        self._target_pub.publish(msg)
        self._last_published_target_idx = self._current_idx

    def _control_cb(self):
        if not self._odom_received or self._done:
            self._cmd_pub.publish(Twist())
            return

        target_x = self._waypoints[self._current_idx]["x"]
        target_y = self._waypoints[self._current_idx]["y"]
        self._publish_current_target_if_changed(target_x, target_y)
        dx = target_x - self._x
        dy = target_y - self._y
        distance      = math.hypot(dx, dy)
        heading_error = _normalize_angle(math.atan2(dy, dx) - self._yaw)

        if self._rotating:
            if abs(heading_error) <= HEADING_DEADBAND:
                self._rotating = False
            else:
                cmd = Twist()
                cmd.angular.z = self._clamp_angular(ANGULAR_GAIN * heading_error)
                self._cmd_pub.publish(cmd)
                return

        if distance < GOAL_TOLERANCE:
            robot_x, robot_y = self._x, self._y
            self.get_logger().info(
                f"Reached waypoint {self._current_idx}: "
                f"type={self._waypoints[self._current_idx]['point_type']} "
                f"target=({target_x:.3f}, {target_y:.3f}) "
                f"robot=({robot_x:.3f}, {robot_y:.3f}) "
                f"dist={distance:.3f}m"
            )
            self._reached_pub.publish(Int32(data=self._current_idx))
            self._current_idx += 1
            self._rotating = True
            if self._current_idx >= len(self._waypoints):
                self._done = True
                self.get_logger().info("All waypoints visited")
            self._cmd_pub.publish(Twist())
            return

        cmd = Twist()
        cmd.linear.x  = LINEAR_SPEED
        cmd.angular.z = self._clamp_angular(ANGULAR_GAIN * heading_error)
        self._cmd_pub.publish(cmd)

    @staticmethod
    def _clamp_angular(value):
        return max(-MAX_ANGULAR_SPEED, min(MAX_ANGULAR_SPEED, value))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
