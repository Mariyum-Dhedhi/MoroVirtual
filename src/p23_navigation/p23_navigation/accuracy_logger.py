#!/usr/bin/env python3
"""Accuracy logger — records ground-truth vs. EKF-filtered pose against the
active waypoint target, for offline tracking-error analysis.

Two outputs:
  * accuracy_log.csv    — one row per 10 Hz tick (continuous trace)
  * reached_events.csv  — one row per /waypoint_reached event, captured
                           directly in that callback so the recorded
                           target_x/target_y is exactly what was active
                           when the robot reported arrival, not whatever
                           /current_target has advanced to by the next
                           10 Hz tick.
"""

import csv

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import PointStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Int32
from tf_transformations import euler_from_quaternion


# Must match waypoint_follower's /current_target publisher QoS: TRANSIENT_LOCAL
# durability is what lets a subscriber started after the first publish still
# receive the last-published target instead of waiting for the next change.
CURRENT_TARGET_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

LOG_PERIOD = 0.1  # s (10 Hz)

LOG_HEADER = [
    "timestamp", "x_gt", "y_gt", "yaw_gt", "x_ekf", "y_ekf",
    "target_x", "target_y", "waypoint_idx", "reached_flag",
]
REACHED_HEADER = [
    "timestamp", "waypoint_idx", "x_gt", "y_gt", "target_x", "target_y",
]


class AccuracyLogger(Node):

    def __init__(self):
        super().__init__("accuracy_logger")

        self.declare_parameter("output_csv", "accuracy_log.csv")
        self.declare_parameter("reached_events_csv", "reached_events.csv")

        output_path = self.get_parameter("output_csv").value
        reached_path = self.get_parameter("reached_events_csv").value

        self._log_file = open(output_path, "w", newline="")
        self._log_writer = csv.writer(self._log_file)
        self._log_writer.writerow(LOG_HEADER)

        self._reached_file = open(reached_path, "w", newline="")
        self._reached_writer = csv.writer(self._reached_file)
        self._reached_writer.writerow(REACHED_HEADER)

        self.get_logger().info(
            f"Logging accuracy trace to {output_path}, "
            f"reached events to {reached_path}")

        self._x_gt = self._y_gt = self._yaw_gt = None
        self._x_ekf = self._y_ekf = None
        self._target_x = self._target_y = None
        self._waypoint_idx = None
        self._reached_flag_pending = False

        self.create_subscription(
            Odometry, "/ground_truth/odom", self._ground_truth_cb, 10)
        self.create_subscription(
            Odometry, "/odometry/filtered", self._ekf_cb, 10)
        self.create_subscription(
            PointStamped, "/current_target", self._current_target_cb,
            CURRENT_TARGET_QOS)
        self.create_subscription(
            Int32, "/waypoint_reached", self._reached_cb, 10)
        self.create_subscription(
            Int32, "/waypoint_follower/current_index", self._current_index_cb, 10)

        self.create_timer(LOG_PERIOD, self._log_cb)

    def _ground_truth_cb(self, msg: Odometry):
        self._x_gt = msg.pose.pose.position.x
        self._y_gt = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, self._yaw_gt = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def _ekf_cb(self, msg: Odometry):
        self._x_ekf = msg.pose.pose.position.x
        self._y_ekf = msg.pose.pose.position.y

    def _current_target_cb(self, msg: PointStamped):
        self._target_x = msg.point.x
        self._target_y = msg.point.y

    def _current_index_cb(self, msg: Int32):
        self._waypoint_idx = msg.data

    def _reached_cb(self, msg: Int32):
        # Write immediately, from this callback, using whatever target_x/
        # target_y and ground-truth pose are currently cached — captured
        # before any later /current_target callback (for the next
        # waypoint) or current_index update can run.
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        self._reached_writer.writerow([
            timestamp, msg.data, self._x_gt, self._y_gt,
            self._target_x, self._target_y,
        ])
        self._reached_file.flush()
        self._reached_flag_pending = True

    def _log_cb(self):
        timestamp = self.get_clock().now().nanoseconds * 1e-9
        reached_flag = 1 if self._reached_flag_pending else 0
        self._reached_flag_pending = False
        self._log_writer.writerow([
            timestamp, self._x_gt, self._y_gt, self._yaw_gt,
            self._x_ekf, self._y_ekf, self._target_x, self._target_y,
            self._waypoint_idx, reached_flag,
        ])
        self._log_file.flush()

    def destroy_node(self):
        self._log_file.close()
        self._reached_file.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AccuracyLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
