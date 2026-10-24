#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Float64
from message_filters import Subscriber, ApproximateTimeSynchronizer

EARTH_RADIUS_M = 6371000.0


class DualGnssHeadingNode(Node):
    def __init__(self):
        super().__init__('dual_gnss_heading_node')

        self.declare_parameter('antenna_baseline', 0.5)
        self.declare_parameter('heading_publish_rate', 10.0)

        self._publish_period = 1.0 / self.get_parameter(
            'heading_publish_rate').value
        self._last_publish_time = None

        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/gnss/heading_pose', 10)
        self._deg_pub = self.create_publisher(Float64, '/gnss/heading_deg', 10)

        self._front_sub = Subscriber(self, NavSatFix, '/gnss/front/fix')
        self._rear_sub = Subscriber(self, NavSatFix, '/gnss/rear/fix')
        self._sync = ApproximateTimeSynchronizer(
            [self._front_sub, self._rear_sub], queue_size=10, slop=0.1)
        self._sync.registerCallback(self._sync_cb)

    def _sync_cb(self, front: NavSatFix, rear: NavSatFix) -> None:
        delta_east, delta_north = self._latlon_to_local_enu(
            rear.latitude, rear.longitude, front.latitude, front.longitude)
        yaw = math.atan2(delta_north, delta_east)

        self.get_logger().info(
            'delta_east=%.4f delta_north=%.4f yaw_deg=%.4f' %
            (delta_east, delta_north, math.degrees(yaw)))

        now = self.get_clock().now()
        if self._last_publish_time is not None:
            elapsed = (now - self._last_publish_time).nanoseconds * 1e-9
            if elapsed < self._publish_period:
                return
        self._last_publish_time = now

        pose_msg = PoseWithCovarianceStamped()
        pose_msg.header.stamp = front.header.stamp
        pose_msg.header.frame_id = 'map'
        pose_msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose_msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        pose_msg.pose.covariance = [0.0] * 36
        pose_msg.pose.covariance[35] = 0.0001
        self._pose_pub.publish(pose_msg)

        deg_msg = Float64()
        deg_msg.data = math.degrees(yaw)
        self._deg_pub.publish(deg_msg)

    @staticmethod
    def _latlon_to_local_enu(ref_lat_deg, ref_lon_deg, lat_deg, lon_deg):
        ref_lat = math.radians(ref_lat_deg)
        ref_lon = math.radians(ref_lon_deg)
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)

        d_lat = lat - ref_lat
        d_lon = lon - ref_lon

        # Haversine great-circle distance between the reference and target points.
        a = (math.sin(d_lat / 2.0) ** 2 +
             math.cos(ref_lat) * math.cos(lat) * math.sin(d_lon / 2.0) ** 2)
        c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
        distance = EARTH_RADIUS_M * c

        bearing = math.atan2(
            math.sin(d_lon) * math.cos(lat),
            math.cos(ref_lat) * math.sin(lat) -
            math.sin(ref_lat) * math.cos(lat) * math.cos(d_lon))

        delta_east = distance * math.sin(bearing)
        delta_north = distance * math.cos(bearing)
        return delta_east, delta_north


def main(args=None):
    rclpy.init(args=args)
    node = DualGnssHeadingNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
