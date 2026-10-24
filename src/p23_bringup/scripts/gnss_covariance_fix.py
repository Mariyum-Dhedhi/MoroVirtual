#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix

_COV = [0.0001, 0.0, 0.0,
        0.0, 0.0001, 0.0,
        0.0, 0.0, 0.0001]


class GnssCovarFix(Node):
    def __init__(self):
        super().__init__('gnss_covariance_fix')
        self._pub = self.create_publisher(NavSatFix, '/gnss/front/fix_cov', 10)
        self.create_subscription(NavSatFix, '/gnss/front/fix', self._cb, 10)

    def _cb(self, msg: NavSatFix) -> None:
        msg.position_covariance = _COV
        msg.position_covariance_type = 1  # COVARIANCE_TYPE_APPROXIMATED
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GnssCovarFix()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
