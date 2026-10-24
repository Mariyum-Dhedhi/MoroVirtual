#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf2_ros
from tf2_ros import TransformException
from tf_transformations import euler_from_quaternion


def _yaw_from_quat(x, y, z, w):
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def _quat_from_yaw(yaw):
    half = yaw * 0.5
    return (0.0, 0.0, math.sin(half), math.cos(half))


class MapFramePublisher(Node):
    def __init__(self):
        super().__init__('map_frame_publisher')
        self._pub = self.create_publisher(Odometry, '/odometry/filtered', 10)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)
        self._imu_yaw = None
        self.create_subscription(Odometry, '/odometry/gps', self._cb, 10)
        self.create_subscription(Odometry, '/odometry/local', self._local_cb, 10)

    def _local_cb(self, msg: Odometry) -> None:
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self._imu_yaw = yaw

    def _cb(self, msg: Odometry) -> None:
        try:
            odom_T_base = self._tf_buffer.lookup_transform(
                'odom', 'base_footprint', rclpy.time.Time()
            )
        except TransformException:
            return

        # Extract GPS pose (desired robot pose in map frame)
        gps_x = msg.pose.pose.position.x
        gps_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        if self._imu_yaw is not None:
            gps_yaw = self._imu_yaw
        else:
            gps_yaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        # Republish GPS odometry relabelled as map-frame pose
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = 'map'
        out.child_frame_id = 'base_footprint'
        out.pose = msg.pose
        out.twist = msg.twist
        if self._imu_yaw is not None:
            hx, hy, hz, hw = _quat_from_yaw(gps_yaw)
            out.pose.pose.orientation.x = hx
            out.pose.pose.orientation.y = hy
            out.pose.pose.orientation.z = hz
            out.pose.pose.orientation.w = hw
        self._pub.publish(out)

        # Extract current odom-frame pose from odom EKF TF
        odom_x = odom_T_base.transform.translation.x
        odom_y = odom_T_base.transform.translation.y
        r = odom_T_base.transform.rotation
        odom_yaw = _yaw_from_quat(r.x, r.y, r.z, r.w)

        # map_T_odom = gps_T_base * inv(odom_T_base)  (proper 2D composition)
        c_yaw = gps_yaw - odom_yaw
        cos_c, sin_c = math.cos(c_yaw), math.sin(c_yaw)
        cx = gps_x - (cos_c * odom_x - sin_c * odom_y)
        cy = gps_y - (sin_c * odom_x + cos_c * odom_y)
        qx, qy, qz, qw = _quat_from_yaw(c_yaw)

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = cx
        t.transform.translation.y = cy
        t.transform.translation.z = 0.0
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self._tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MapFramePublisher()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
