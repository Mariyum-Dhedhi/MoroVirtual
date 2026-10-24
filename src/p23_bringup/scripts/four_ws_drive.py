#!/usr/bin/env python3
"""4-Wheel Steering controller — bicycle model.

Kinematic formula (single-track, applied per axle):

  steer_front = atan2( vy + wz·L,  vx )   (velocity direction at front axle centre)
  steer_rear  = atan2( vy − wz·L,  vx )   (velocity direction at rear  axle centre)

where L = HALF_WHEELBASE.  Both front wheels share steer_front; both rear share steer_rear.
Angles outside (−π/2, π/2] are folded — equivalent to flipping the wheel 180° and reversing
the drive direction — so the model works correctly for backward motion too.

cmd_vel → behaviour:

  vx > 0, wz = 0           straight forward           steer_f =  steer_r = 0°
  vx > 0, vy = 0, wz > 0   left arc (car-like)        steer_f > 0, steer_r < 0
  vx > 0, vy = 0, wz < 0   right arc                  steer_f < 0, steer_r > 0
  vx < 0, vy = 0, wz > 0   backward left arc          steer_f < 0, steer_r > 0
  vx > 0, vy > 0, wz = 0   forward-left crab          steer_f = steer_r = atan2(vy,vx)
  vx > 0, vy > 0, wz > 0   forward-left + left arc    steer_f > steer_r
  vx = 0, vy = 0, wz ≠ 0   spin in place              (full kinematic)
  vx = 0, vy ≠ 0, wz = 0   crab                       (full kinematic → all wheels ±90°)

Wheel order matches controllers.yaml:
  [front_left, front_right, rear_left, rear_right]
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


# ── Robot geometry — must match _macros.xacro ─────────────────────────
HALF_WHEELBASE = 1.7 / 2.0   # 0.85 m
HALF_TRACK     = 2.0 / 2.0   # 1.00 m
WHEEL_RADIUS   = 0.5          # m

# (x, y) in base_link; same order as controllers.yaml joints
WHEEL_XY = [
    ( HALF_WHEELBASE,  HALF_TRACK),   # front_left
    ( HALF_WHEELBASE, -HALF_TRACK),   # front_right
    (-HALF_WHEELBASE,  HALF_TRACK),   # rear_left
    (-HALF_WHEELBASE, -HALF_TRACK),   # rear_right
]

# /joint_states names, same order as WHEEL_XY
STEER_JOINTS = ["front_left_steer_joint", "front_right_steer_joint",
                "rear_left_steer_joint", "rear_right_steer_joint"]
WHEEL_JOINTS = ["front_left_wheel_joint", "front_right_wheel_joint",
                "rear_left_wheel_joint", "rear_right_wheel_joint"]

# /odom twist covariance (row-major 6x6, order x,y,z,rx,ry,rz).
# vx comes from forward kinematics and is largely slip-free -> tight
# variance, high trust. vyaw is heavily affected by real wheel slip during
# turns -> loose variance, so ekf_filter_node_odom leans on /imu/data's
# much tighter vyaw covariance instead, while /odom still contributes vx
# and acts as a supporting signal for vyaw.
ODOM_TWIST_COVARIANCE = [0.0] * 36
ODOM_TWIST_COVARIANCE[0]  = 0.01   # vx variance, (m/s)^2
ODOM_TWIST_COVARIANCE[35] = 0.5    # vyaw variance, (rad/s)^2

VX_DEADBAND = 0.01
HALF_PI     = math.pi / 2.0

# Fraction of HALF_WHEELBASE used in the steer formula.
# 1.0 → full kinematic angle (≈ ±60° at default teleop speed).
# 0.5 → half the angle (≈ ±40°), wider, more car-like arcs.
STEER_GAIN  = 0.2


def _fold(angle: float):
    """Fold angle into (−π/2, π/2]; return folded angle (drive sign is implicit in vx)."""
    if angle > HALF_PI:
        return angle - math.pi
    if angle <= -HALF_PI:
        return angle + math.pi
    return angle


class FourWSDrive(Node):

    def __init__(self):
        super().__init__("four_ws_drive")

        self._steer_pub = self.create_publisher(
            Float64MultiArray, "/steer_controller/commands", 10)
        self._drive_pub = self.create_publisher(
            Float64MultiArray, "/drive_controller/commands", 10)
        self._odom_pub  = self.create_publisher(Odometry, "/odom", 10)
        self._tf_br     = TransformBroadcaster(self)

        # odom->base_footprint TF is owned by ekf_filter_node_odom; this
        # controller only broadcasts it if explicitly enabled.
        self.declare_parameter("publish_tf", False)
        self._publish_tf = self.get_parameter("publish_tf").value

        self.create_subscription(Twist, "/cmd_vel", self._cmd_vel_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_state_cb, 10)

        self._vx = self._vy = self._wz = 0.0
        self._x  = self._y  = self._yaw = 0.0
        self._steer_pos = [0.0] * 4
        self._wheel_vel = [0.0] * 4
        self._last_time = self.get_clock().now()

        self.create_timer(0.02, self._odom_cb)

    # ── /cmd_vel ───────────────────────────────────────────────────────
    def _cmd_vel_cb(self, msg: Twist):
        vx = msg.linear.x
        vy = msg.linear.y
        wz = msg.angular.z

        steers, speeds = self._kinematics(vx, vy, wz)

        s = Float64MultiArray(); s.data = steers
        d = Float64MultiArray(); d.data = speeds
        self._steer_pub.publish(s)
        self._drive_pub.publish(d)

    # ── /joint_states — actual (post-physics) wheel feedback ────────────
    def _joint_state_cb(self, msg: JointState):
        name_to_idx = {n: i for i, n in enumerate(msg.name)}

        for i, jn in enumerate(STEER_JOINTS):
            idx = name_to_idx.get(jn)
            if idx is not None:
                self._steer_pos[i] = msg.position[idx]

        for i, jn in enumerate(WHEEL_JOINTS):
            idx = name_to_idx.get(jn)
            if idx is not None and idx < len(msg.velocity):
                self._wheel_vel[i] = msg.velocity[idx]

        # /odom should reflect the rotation rate the wheels are actually
        # achieving (per physics/encoder feedback), not the commanded
        # request, so it agrees with what the IMU measures.
        self._vx, self._vy, self._wz = self._forward_kinematics(
            self._steer_pos, self._wheel_vel)

    # ── Kinematics ─────────────────────────────────────────────────────
    def _kinematics(self, vx, vy, wz):
        steers = [0.0] * 4
        speeds = [0.0] * 4

        if abs(vx) >= VX_DEADBAND:
            # ── BICYCLE MODEL ──────────────────────────────────────────
            #
            # Steer angle = direction of velocity at each axle midpoint.
            #
            #   Front (x = +HL): vel = (vx,  vy + wz·HL)
            #   Rear  (x = −HL): vel = (vx,  vy − wz·HL)
            #
            # Fold into (−π/2, π/2] so joints stay within limits.
            # When vx < 0 the raw angle is in (π/2, π] or (−π, −π/2];
            # folding flips the steer and the copysign(·, vx) on drive
            # speed jointly achieve the correct backward motion.

            L = HALF_WHEELBASE * STEER_GAIN
            steer_f = _fold(math.atan2(vy + wz * L, vx))
            steer_r = _fold(math.atan2(vy - wz * L, vx))

            steers[0] = steer_f   # front_left
            steers[1] = steer_f   # front_right
            steers[2] = steer_r   # rear_left
            steers[3] = steer_r   # rear_right

            # Per-wheel drive speed: full kinematic gives the correct
            # differential between inner / outer wheels.
            # Sign is taken from vx: negative → all wheels drive backward.
            for i, (xi, yi) in enumerate(WHEEL_XY):
                wx = vx - wz * yi
                wy = vy + wz * xi
                mag = math.sqrt(wx * wx + wy * wy)
                speeds[i] = math.copysign(mag, vx) / WHEEL_RADIUS

        else:
            # ── FULL KINEMATIC (spin in place or crab) ─────────────────
            for i, (xi, yi) in enumerate(WHEEL_XY):
                wx = vx - wz * yi
                wy = vy + wz * xi
                if abs(wx) < 1e-6 and abs(wy) < 1e-6:
                    continue
                angle = math.atan2(wy, wx)
                if angle > HALF_PI:
                    angle -= math.pi; sign = -1.0
                elif angle <= -HALF_PI:
                    angle += math.pi; sign = -1.0
                else:
                    sign = 1.0
                steers[i] = angle
                speeds[i] = math.sqrt(wx * wx + wy * wy) / WHEEL_RADIUS * sign

        return steers, speeds

    # ── Forward kinematics — inverse of _kinematics() ──────────────────
    # Recovers the body twist implied by the commanded per-wheel steer
    # angles + drive speeds via least squares (4 wheels, 3 unknowns).
    # WHEEL_XY's left/right and front/rear symmetry makes J^T J diagonal,
    # giving this closed form (no matrix inverse needed).
    def _forward_kinematics(self, steers, speeds):
        sum_u = sum_w = sum_cross = 0.0
        for (xi, yi), steer, speed in zip(WHEEL_XY, steers, speeds):
            ground_speed = speed * WHEEL_RADIUS
            u = ground_speed * math.cos(steer)
            w = ground_speed * math.sin(steer)
            sum_u += u
            sum_w += w
            sum_cross += -yi * u + xi * w

        vx = sum_u / 4.0
        vy = sum_w / 4.0
        wz = sum_cross / (4.0 * (HALF_WHEELBASE ** 2 + HALF_TRACK ** 2))
        return vx, vy, wz

    # ── Odometry — dead-reckoning at 50 Hz ────────────────────────────
    def _odom_cb(self):
        now = self.get_clock().now()
        dt  = (now - self._last_time).nanoseconds * 1e-9
        self._last_time = now

        c, s = math.cos(self._yaw), math.sin(self._yaw)
        self._x   += (self._vx * c - self._vy * s) * dt
        self._y   += (self._vx * s + self._vy * c) * dt
        self._yaw += self._wz * dt

        qz = math.sin(self._yaw / 2.0)
        qw = math.cos(self._yaw / 2.0)
        stamp = now.to_msg()

        odom = Odometry()
        odom.header.stamp          = stamp
        odom.header.frame_id       = "odom"
        odom.child_frame_id        = "base_footprint"
        odom.pose.pose.position.x  = self._x
        odom.pose.pose.position.y  = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = self._vx
        odom.twist.twist.linear.y  = self._vy
        odom.twist.twist.angular.z = self._wz
        odom.twist.covariance     = ODOM_TWIST_COVARIANCE
        self._odom_pub.publish(odom)

        if self._publish_tf:
            tf = TransformStamped()
            tf.header.stamp    = stamp
            tf.header.frame_id = "odom"
            tf.child_frame_id  = "base_footprint"
            tf.transform.translation.x = self._x
            tf.transform.translation.y = self._y
            tf.transform.rotation.z    = qz
            tf.transform.rotation.w    = qw
            self._tf_br.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = FourWSDrive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
