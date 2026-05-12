"""Ranger Mini v3 sim messenger.

Mirrors the ROS interface of ranger_base/src/ranger_messenger.cpp.
Subscribes to /cmd_vel and publishes /odom + per-wheel controller
commands. Round 07 implements DUAL_ACKERMAN only; other modes
set the mode and warn.

Kinematic constants come from RangerMiniV3Params in the real
driver's ranger_params.hpp.
"""

from dataclasses import dataclass
from enum import IntEnum
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster


# ============================================================
# Constants — from RangerMiniV3Params (real-driver source of truth)
# ============================================================
WHEELBASE   = 0.494                       # m
TRACK       = 0.364                       # m
WHEEL_RADIUS = 0.09                       # m  (from URDF; not in params)
MAX_LINEAR_SPEED = 1.5                    # m/s
MAX_ANGULAR_SPEED = 4.8                   # rad/s
MIN_TURN_RADIUS = 0.4764                  # m
MAX_STEER_ACKERMANN = 0.601               # rad
MAX_STEER_PARALLEL  = 1.570               # rad


class MotionMode(IntEnum):
    """Mirrors ranger_msgs/msg/MotionState constants."""
    DUAL_ACKERMAN = 0
    PARALLEL      = 1
    SPINNING      = 2
    SIDE_SLIP     = 3


@dataclass
class WheelCommands:
    """Eight scalars: 4 steering angles + 4 wheel velocities.
    Order: fl, fr, rl, rr.
    """
    steer_fl: float = 0.0
    steer_fr: float = 0.0
    steer_rl: float = 0.0
    steer_rr: float = 0.0
    vel_fl: float = 0.0
    vel_fr: float = 0.0
    vel_rl: float = 0.0
    vel_rr: float = 0.0


# ============================================================
# Kinematic helpers (ported from ranger_messenger.cpp)
# ============================================================
def calculate_steering_angle(linear_x: float, angular_z: float):
    """Port of CalculateSteeringAngle.

    Returns (inner_wheel_angle, turn_radius).
    Sign: positive inner-wheel angle = left turn for forward motion.
    """
    lin = abs(linear_x)
    ang = abs(angular_z)
    if ang < 1e-6:
        return 0.0, math.inf
    radius = lin / ang
    k = 1 if (angular_z * linear_x) >= 0 else -1
    phi_i = math.atan((WHEELBASE / 2.0) / radius)
    phi_i = min(phi_i, math.radians(40.0))
    return k * phi_i, radius


def inner_to_central(angle: float):
    """Port of ConvertInnerAngleToCentral.

    Translates an inner-wheel Ackermann angle to the equivalent
    bicycle-model central angle (used by the wheelbase-only
    forward-kinematics model).
    """
    phi_i = abs(angle)
    phi = math.atan(
        WHEELBASE * math.sin(phi_i) /
        (WHEELBASE * math.cos(phi_i) + TRACK * math.sin(phi_i))
    )
    return phi if angle >= 0 else -phi


def per_wheel_steering_dual_ackermann(inner_phi: float):
    """Compute the four wheel steering angles in DUAL_ACKERMAN.

    Geometry: front and rear axles mirror each other; left and
    right wheels on each axle are NOT the same (Ackermann
    geometry). The 'inner' wheel (inside the turn) is at the
    larger angle.

    Returns (fl, fr, rl, rr) in radians, each per the URDF
    steering joint axis convention (axis (0,0,-1)).
    """
    if abs(inner_phi) < 1e-6:
        return 0.0, 0.0, 0.0, 0.0

    # k > 0: turning left (positive z rotation of the vehicle).
    # Inside wheels are LEFT side. Outside wheels are RIGHT side.
    sign = 1.0 if inner_phi > 0 else -1.0
    phi_i = abs(inner_phi)

    # Radius from the vehicle center to its instantaneous center of
    # rotation (ICR), computed from the inner angle. The dual-
    # Ackermann assumption is that the ICR sits on the y axis at
    # the vehicle center; both axles steer toward it symmetrically.
    R = (WHEELBASE / 2.0) / math.tan(phi_i)

    # Outer wheel angle: larger turn radius, smaller angle.
    phi_o = math.atan((WHEELBASE / 2.0) / (R + TRACK))

    # Assign: inside (left) wheels get phi_i, outside (right) get phi_o.
    # Front axle: positive steer; rear axle: negative steer (mirror).
    if sign > 0:
        fl, fr, rl, rr = phi_i, phi_o, -phi_i, -phi_o
    else:
        fl, fr, rl, rr = -phi_o, -phi_i, phi_o, phi_i

    return fl, fr, rl, rr


def compute_wheel_commands_dual_ackermann(linear_x: float, inner_phi: float):
    """Compute all 8 commands for DUAL_ACKERMAN mode.

    For wheel velocities Round 07 uses the simplification of
    commanding all four wheels to the same angular velocity:
      w = linear_x / wheel_radius
    This is correct in straight lines; in tight turns each wheel
    should rotate at a slightly different rate. Round 08 may
    refine if visible slip becomes a problem.
    """
    fl_s, fr_s, rl_s, rr_s = per_wheel_steering_dual_ackermann(inner_phi)
    w = linear_x / WHEEL_RADIUS

    return WheelCommands(
        steer_fl=fl_s, steer_fr=fr_s, steer_rl=rl_s, steer_rr=rr_s,
        vel_fl=w, vel_fr=w, vel_rl=w, vel_rr=w,
    )


# ============================================================
# The node
# ============================================================
class SimMessenger(Node):
    """Twist-to-controllers messenger for the Ranger Mini v3 sim."""

    def __init__(self):
        super().__init__('sim_messenger')

        # Parameters
        self.declare_parameter('update_rate', 50)        # Hz
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic_name', 'odom')
        self.declare_parameter('publish_odom_tf', False)

        self.update_rate = self.get_parameter('update_rate').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_topic = self.get_parameter('odom_topic_name').value
        self.publish_odom_tf = self.get_parameter('publish_odom_tf').value

        # State
        self.motion_mode = MotionMode.DUAL_ACKERMAN
        self.last_twist = Twist()
        self.position_x = 0.0
        self.position_y = 0.0
        self.theta = 0.0
        self.last_time = None
        self.last_inner_phi = 0.0   # remember for odometry integration
        self._mode_warned = None

        # QoS: BestEffort for /cmd_vel (matches typical teleop pubs),
        # Reliable for /odom (downstream usually needs every sample).
        cmd_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Pub/Sub
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_cb, cmd_qos
        )
        self.odom_pub = self.create_publisher(
            Odometry, self.odom_topic, odom_qos
        )
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_odom_tf else None

        # Controller command publishers — one per joint.
        # Each controller takes Float64MultiArray with one element
        # (its single joint, per controllers.yaml).
        self.steer_pubs = {
            'fl': self.create_publisher(Float64MultiArray, '/fl_steering_position_controller/commands', 10),
            'fr': self.create_publisher(Float64MultiArray, '/fr_steering_position_controller/commands', 10),
            'rl': self.create_publisher(Float64MultiArray, '/rl_steering_position_controller/commands', 10),
            'rr': self.create_publisher(Float64MultiArray, '/rr_steering_position_controller/commands', 10),
        }
        self.wheel_pubs = {
            'fl': self.create_publisher(Float64MultiArray, '/fl_wheel_velocity_controller/commands', 10),
            'fr': self.create_publisher(Float64MultiArray, '/fr_wheel_velocity_controller/commands', 10),
            'rl': self.create_publisher(Float64MultiArray, '/rl_wheel_velocity_controller/commands', 10),
            'rr': self.create_publisher(Float64MultiArray, '/rr_wheel_velocity_controller/commands', 10),
        }

        # Timer
        self.timer = self.create_timer(1.0 / self.update_rate, self._tick)

        self.get_logger().info(
            f'SimMessenger up. update_rate={self.update_rate} Hz, '
            f'publish_odom_tf={self.publish_odom_tf}'
        )

    # --------------------------------------------------------
    # Twist callback: only updates mode + stores latest Twist
    # --------------------------------------------------------
    def _cmd_cb(self, msg: Twist):
        self.last_twist = msg

        # Mode selection — port of TwistCmdCallback (lines 388-414)
        # NOTE: the v1-side-slip branch is skipped (we're v3-only).
        if msg.linear.y != 0.0:
            self.motion_mode = MotionMode.PARALLEL
        else:
            _, radius = calculate_steering_angle(msg.linear.x, msg.angular.z)
            if radius < MIN_TURN_RADIUS:
                self.motion_mode = MotionMode.SPINNING
            else:
                self.motion_mode = MotionMode.DUAL_ACKERMAN

    # --------------------------------------------------------
    # Timer tick: compute & publish wheel commands + odometry
    # --------------------------------------------------------
    def _tick(self):
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        msg = self.last_twist
        wc = WheelCommands()    # default zeros

        if self.motion_mode == MotionMode.DUAL_ACKERMAN:
            # Compute steering and wheel velocity.
            inner_phi, _radius = calculate_steering_angle(
                msg.linear.x, msg.angular.z
            )
            # Clamp to max Ackermann angle (per ranger_messenger.cpp L419)
            inner_phi = max(-MAX_STEER_ACKERMANN,
                            min(MAX_STEER_ACKERMANN, inner_phi))
            self.last_inner_phi = inner_phi
            wc = compute_wheel_commands_dual_ackermann(msg.linear.x, inner_phi)

            # Odometry integration: bicycle-model with central angle.
            central = inner_to_central(inner_phi)
            v = msg.linear.x
            # RK4 step (10 substeps for accuracy)
            self._integrate_dual_ackermann(v, central, dt)

        else:
            # PARALLEL and SPINNING modes are recognized but their
            # outputs are not wired yet. Zero commands + a throttled
            # log so we don't spam.
            if self._mode_warned != self.motion_mode:
                self.get_logger().warn(
                    f'Mode {self.motion_mode.name} recognized but not '
                    f'yet implemented in Round 07. Wheels zeroed.'
                )
                self._mode_warned = self.motion_mode
            self.last_inner_phi = 0.0

        self._publish_wheel_commands(wc)
        self._publish_odometry(msg, now)
        # Reset the suppression flag when we leave a non-implemented mode
        if self.motion_mode == MotionMode.DUAL_ACKERMAN:
            self._mode_warned = None

    # --------------------------------------------------------
    def _integrate_dual_ackermann(self, v: float, phi: float, dt: float):
        """RK4 of DualAckermanModel from kinematics_model.hpp.

        State: (x, y, theta). Control: (v, phi) where phi is the
        central (bicycle) angle.
        """
        def f(state, _t):
            x, y, th = state
            return [
                v * math.cos(phi) * math.cos(th),
                v * math.cos(phi) * math.sin(th),
                2.0 * v * math.sin(phi) / WHEELBASE,
            ]

        state = [self.position_x, self.position_y, self.theta]
        # 10 RK4 substeps
        h = dt / 10.0
        t = 0.0
        for _ in range(10):
            k1 = f(state, t)
            s2 = [state[i] + 0.5 * h * k1[i] for i in range(3)]
            k2 = f(s2, t + 0.5 * h)
            s3 = [state[i] + 0.5 * h * k2[i] for i in range(3)]
            k3 = f(s3, t + 0.5 * h)
            s4 = [state[i] + h * k3[i] for i in range(3)]
            k4 = f(s4, t + h)
            state = [
                state[i] + (h / 6.0) * (k1[i] + 2*k2[i] + 2*k3[i] + k4[i])
                for i in range(3)
            ]
            t += h
        self.position_x, self.position_y, self.theta = state

    # --------------------------------------------------------
    def _publish_wheel_commands(self, wc: WheelCommands):
        def pub_one(p, value):
            m = Float64MultiArray()
            m.data = [float(value)]
            p.publish(m)

        pub_one(self.steer_pubs['fl'], wc.steer_fl)
        pub_one(self.steer_pubs['fr'], wc.steer_fr)
        pub_one(self.steer_pubs['rl'], wc.steer_rl)
        pub_one(self.steer_pubs['rr'], wc.steer_rr)

        pub_one(self.wheel_pubs['fl'], wc.vel_fl)
        pub_one(self.wheel_pubs['fr'], wc.vel_fr)
        pub_one(self.wheel_pubs['rl'], wc.vel_rl)
        pub_one(self.wheel_pubs['rr'], wc.vel_rr)

    # --------------------------------------------------------
    def _publish_odometry(self, last_cmd: Twist, now):
        quat = self._yaw_to_quat(self.theta)

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id  = self.base_frame
        odom.pose.pose.position.x = self.position_x
        odom.pose.pose.position.y = self.position_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat

        # Twist on /odom matches the real driver's convention:
        # publish the COMMANDED twist, not measured wheel-state.
        if self.motion_mode == MotionMode.DUAL_ACKERMAN:
            central = inner_to_central(self.last_inner_phi)
            odom.twist.twist.linear.x = last_cmd.linear.x
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = (
                2.0 * last_cmd.linear.x * math.sin(central) / WHEELBASE
            )
        else:
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.angular.z = 0.0

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            tf = TransformStamped()
            tf.header.stamp = now.to_msg()
            tf.header.frame_id = self.odom_frame
            tf.child_frame_id  = self.base_frame
            tf.transform.translation.x = self.position_x
            tf.transform.translation.y = self.position_y
            tf.transform.translation.z = 0.0
            tf.transform.rotation = quat
            self.tf_broadcaster.sendTransform(tf)

    # --------------------------------------------------------
    @staticmethod
    def _yaw_to_quat(yaw: float) -> Quaternion:
        q = Quaternion()
        q.z = math.sin(yaw * 0.5)
        q.w = math.cos(yaw * 0.5)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = SimMessenger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
