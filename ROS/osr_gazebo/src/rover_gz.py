#!/usr/bin/env python3
import math
from functools import partial

import rclpy
from rclpy.parameter import Parameter
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64, Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import tf2_ros
from geometry_msgs.msg import TransformStamped


class RoverGazebo(Node):
    """
    Gazebo simulation equivalent of rover.py.
    Same math and control logic, adapted for Gazebo interfaces:
      - /wheel_controller/commands      (Float64MultiArray) instead of /cmd_drive + RoboClaw driver
      - /servo_controller/joint_trajectory (JointTrajectory) instead of /cmd_corner + servo_wrapper
      - /joint_states                   instead of /drive_state + /corner_state
    """

    # Joint names as published by Gazebo's joint_state_broadcaster
    DRIVE_JOINTS = [
        'front_wheel_joint_left',
        'front_wheel_joint_right',
        'middle_wheel_joint_left',
        'middle_wheel_joint_right',
        'rear_wheel_joint_left',
        'rear_wheel_joint_right',
    ]
    CORNER_JOINTS = [
        'front_wheel_joint_L',   # left front  servo
        'front_wheel_joint_R',   # right front servo
        'rear_wheel_joint_L',    # left rear   servo
        'rear_wheel_joint_R',    # right rear  servo
    ]
    # Order expected by /wheel_controller/commands (must match controller_velocity.yaml)
    WHEEL_CMD_ORDER = [
        'middle_wheel_joint_left',
        'middle_wheel_joint_right',
        'front_wheel_joint_left',
        'front_wheel_joint_right',
        'rear_wheel_joint_left',
        'rear_wheel_joint_right',
    ]
    # Order expected by /servo_controller (joint_names in JointTrajectory)
    SERVO_CMD_ORDER = [
        'front_wheel_joint_R',
        'front_wheel_joint_L',
        'rear_wheel_joint_R',
        'rear_wheel_joint_L',
    ]

    def __init__(self):
        super().__init__("rover_gazebo")
        self.log = self.get_logger()
        self.log.info("Initializing RoverGazebo node")

        self.declare_parameters(
            namespace='',
            parameters=[
                ('rover_dimensions.d1', Parameter.Type.DOUBLE),
                ('rover_dimensions.d2', Parameter.Type.DOUBLE),
                ('rover_dimensions.d3', Parameter.Type.DOUBLE),
                ('rover_dimensions.d4', Parameter.Type.DOUBLE),
                ('rover_dimensions.wheel_radius', Parameter.Type.DOUBLE),
                ('drive_no_load_rpm', Parameter.Type.DOUBLE),
                ('enable_odometry', Parameter.Type.BOOL),
                ('publish_transform', Parameter.Type.BOOL),
            ]
        )

        self.d1 = self.get_parameter('rover_dimensions.d1').get_parameter_value().double_value
        self.d2 = self.get_parameter('rover_dimensions.d2').get_parameter_value().double_value
        self.d3 = self.get_parameter('rover_dimensions.d3').get_parameter_value().double_value
        self.d4 = self.get_parameter('rover_dimensions.d4').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('rover_dimensions.wheel_radius').get_parameter_value().double_value
        drive_no_load_rpm = self.get_parameter('drive_no_load_rpm').get_parameter_value().double_value
        self.max_vel = self.wheel_radius * drive_no_load_rpm / 60 * 2 * math.pi  # [m/s]
        self.should_calculate_odom = self.get_parameter('enable_odometry').get_parameter_value().bool_value
        self.should_publish_transform = self.get_parameter('publish_transform').get_parameter_value().bool_value

        self.min_radius = 0.45   # [m]
        self.max_radius = 6.4    # [m]
        self.no_cmd_thresh = 0.05  # [rad]

        # Current joint state storage (populated from /joint_states)
        self.curr_positions = {}   # joint_name -> position [rad]
        self.curr_velocities = {}  # joint_name -> velocity [rad/s]
        self.curr_turning_radius = self.max_radius

        # Odometry
        if self.should_calculate_odom:
            self.log.info("Odometry enabled, publishing to /odom")
            self.odometry = Odometry()
            self.odometry.header.frame_id = "odom"
            self.odometry.child_frame_id = "base_link"
            self.odometry.header.stamp = self.get_clock().now().to_msg()
            self.odometry.pose.pose.orientation.w = 1.0
            if self.should_publish_transform:
                self.tf_pub = tf2_ros.TransformBroadcaster(self)

        # ---------- Subscribers ----------
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', partial(self.cmd_cb, intuitive=False), 1)
        self.cmd_vel_int_sub = self.create_subscription(
            Twist, '/cmd_vel_intuitive', partial(self.cmd_cb, intuitive=True), 1)
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_cb, 1)

        # ---------- Publishers ----------
        # Drive motors: Float64MultiArray in WHEEL_CMD_ORDER
        self.wheel_pub = self.create_publisher(
            Float64MultiArray, '/wheel_controller/commands', 1)
        # Corner servos: JointTrajectory
        self.servo_pub = self.create_publisher(
            JointTrajectory, '/servo_controller/joint_trajectory', 1)
        # Turning radius (debug / info)
        self.turning_radius_pub = self.create_publisher(Float64, '/turning_radius', 1)
        if self.should_calculate_odom:
            self.odometry_pub = self.create_publisher(Odometry, '/odom', 2)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def joint_state_cb(self, msg: JointState):
        """Receive joint states from Gazebo and update internal state."""
        for name, pos, vel in zip(msg.name, msg.position, msg.velocity):
            self.curr_positions[name] = pos
            self.curr_velocities[name] = vel

        # Only compute odometry once we have all 10 joints (6 drive + 4 corner)
        all_joints = self.DRIVE_JOINTS + self.CORNER_JOINTS
        if self.should_calculate_odom and all(j in self.curr_positions for j in all_joints):
            now = self.get_clock().now()
            dt = float(
                now.nanoseconds - (
                    self.odometry.header.stamp.sec * 10**9
                    + self.odometry.header.stamp.nanosec
                )
            ) / 10**9
            if dt <= 0:
                return
            self._forward_kinematics()
            dx = self.odometry.twist.twist.linear.x * dt
            dth = self.odometry.twist.twist.angular.z * dt
            current_angle = 2 * math.atan2(
                self.odometry.pose.pose.orientation.z,
                self.odometry.pose.pose.orientation.w
            )
            new_angle = current_angle + dth
            self.odometry.pose.pose.orientation.z = math.sin(new_angle / 2.)
            self.odometry.pose.pose.orientation.w = math.cos(new_angle / 2.)
            self.odometry.pose.pose.position.x += math.cos(new_angle) * dx
            self.odometry.pose.pose.position.y += math.sin(new_angle) * dx
            self.odometry.pose.covariance = [0.0] * 36
            self.odometry.twist.covariance[0]  = 0.0225
            self.odometry.twist.covariance[5]  = 0.01
            self.odometry.twist.covariance[-5] = 0.0225
            self.odometry.twist.covariance[-1] = 0.04
            self.odometry.header.stamp = now.to_msg()
            self.odometry_pub.publish(self.odometry)

            if self.should_publish_transform:
                t = TransformStamped()
                t.header.stamp = now.to_msg()
                t.header.frame_id = "odom"
                t.child_frame_id = "base_link"
                t.transform.translation.x = self.odometry.pose.pose.position.x
                t.transform.translation.y = self.odometry.pose.pose.position.y
                t.transform.rotation = self.odometry.pose.pose.orientation
                self.tf_pub.sendTransform(t)

    def cmd_cb(self, twist_msg: Twist, intuitive=False):
        """Convert a Twist command into wheel velocities and servo angles."""
        # Rotate in place: angular.y used as in rover.py
        if twist_msg.angular.y and not twist_msg.linear.x:
            corner_angles, wheel_vels = self._calculate_rotate_in_place(twist_msg)
        else:
            radius = self._twist_to_turning_radius(twist_msg, intuitive_mode=intuitive)
            self.log.debug(f"Turning radius: {radius:.3f} m", throttle_duration_sec=1)

            corner_angles = self._calculate_corner_angles(radius)

            max_vel = abs(radius) / (abs(radius) + self.d1) * self.max_vel
            if math.isnan(max_vel):
                max_vel = self.max_vel
            velocity = min(max_vel, twist_msg.linear.x)
            wheel_vels = self._calculate_drive_velocities(velocity, radius)

        self._publish_wheel_velocities(wheel_vels)
        self._publish_servo_angles(corner_angles)

    # ------------------------------------------------------------------
    # Kinematics — identical logic to rover.py
    # ------------------------------------------------------------------

    def _twist_to_turning_radius(self, twist: Twist, clip=True, intuitive_mode=False) -> float:
        try:
            if intuitive_mode and twist.linear.x < 0:
                radius = twist.linear.x / -twist.angular.z
            else:
                radius = twist.linear.x / twist.angular.z
        except ZeroDivisionError:
            return float('inf')

        if not clip:
            return radius
        if radius == 0:
            if intuitive_mode:
                if twist.angular.z == 0:
                    return self.max_radius
                else:
                    radius = self.min_radius * self.max_vel / twist.angular.z
            else:
                return self.max_radius
        if radius > 0:
            radius = max(self.min_radius, min(self.max_radius, radius))
        else:
            radius = max(-self.max_radius, min(-self.min_radius, radius))
        return radius

    def _calculate_corner_angles(self, radius: float) -> dict:
        """
        Returns a dict: servo_joint_name -> angle [rad]
        Positive angle = right turn (z down frame, same as rover.py)
        """
        angles = {j: 0.0 for j in self.CORNER_JOINTS}
        if abs(radius) >= self.max_radius:
            return angles  # straight, all zeros

        theta_closest  = math.atan2(self.d3, abs(radius) - self.d1)
        theta_farthest = math.atan2(self.d3, abs(radius) + self.d1)

        if radius > 0:  # turning left
            angles['front_wheel_joint_L'] = -theta_closest
            angles['front_wheel_joint_R'] = -theta_farthest
            angles['rear_wheel_joint_L']  =  theta_closest
            angles['rear_wheel_joint_R']  =  theta_farthest
        else:            # turning right
            angles['front_wheel_joint_L'] =  theta_farthest
            angles['front_wheel_joint_R'] =  theta_closest
            angles['rear_wheel_joint_L']  = -theta_farthest
            angles['rear_wheel_joint_R']  = -theta_closest
        return angles

    def _calculate_drive_velocities(self, speed: float, radius: float) -> dict:
        """
        Returns a dict: drive_joint_name -> angular velocity [rad/s]
        Right-side wheels get negative velocity (they face the other way).
        """
        vels = {j: 0.0 for j in self.DRIVE_JOINTS}
        speed = max(-self.max_vel, min(self.max_vel, speed))
        if speed == 0:
            return vels

        if abs(radius) >= self.max_radius:  # straight
            w = speed / self.wheel_radius
            for j in self.DRIVE_JOINTS:
                vels[j] = w if 'left' in j else -w
            return vels

        r = abs(radius)
        w_center = speed / r
        vel_ml = (r - self.d4) * w_center
        vel_cl = math.hypot(r - self.d1, self.d3) * w_center
        vel_cr = math.hypot(r + self.d1, self.d3) * w_center
        vel_mr = (r + self.d4) * w_center

        ang_ml = vel_ml / self.wheel_radius
        ang_cl = vel_cl / self.wheel_radius
        ang_cr = vel_cr / self.wheel_radius
        ang_mr = vel_mr / self.wheel_radius

        if radius > 0:  # left turn: left = closest, right = farthest
            vels['front_wheel_joint_left']   =  ang_cl
            vels['rear_wheel_joint_left']    =  ang_cl
            vels['middle_wheel_joint_left']  =  ang_ml
            vels['front_wheel_joint_right']  = -ang_cr
            vels['rear_wheel_joint_right']   = -ang_cr
            vels['middle_wheel_joint_right'] = -ang_mr
        else:            # right turn: right = closest, left = farthest
            vels['front_wheel_joint_left']   =  ang_cr
            vels['rear_wheel_joint_left']    =  ang_cr
            vels['middle_wheel_joint_left']  =  ang_mr
            vels['front_wheel_joint_right']  = -ang_cl
            vels['rear_wheel_joint_right']   = -ang_cl
            vels['middle_wheel_joint_right'] = -ang_ml
        return vels

    def _calculate_rotate_in_place(self, twist: Twist):
        """Corner angles and wheel velocities for in-place rotation."""
        angles = {
            'front_wheel_joint_L': math.atan2(self.d3, self.d1),
            'front_wheel_joint_R': -math.atan2(self.d3, self.d1),
            'rear_wheel_joint_L':  -math.atan2(self.d2, self.d1),
            'rear_wheel_joint_R':  math.atan2(self.d2, self.d1),
        }
        w = twist.angular.y
        front_vel = math.hypot(self.d1, self.d3) * w / self.wheel_radius
        back_vel  = math.hypot(self.d1, self.d2) * w / self.wheel_radius
        mid_vel   = self.d4 * w / self.wheel_radius
        vels = {
            'front_wheel_joint_left':   front_vel,
            'front_wheel_joint_right':  front_vel,
            'rear_wheel_joint_left':    back_vel,
            'rear_wheel_joint_right':   back_vel,
            'middle_wheel_joint_left':  mid_vel,
            'middle_wheel_joint_right': mid_vel,
        }
        return angles, vels

    def _forward_kinematics(self):
        """
        Estimate current twist from joint states.
        Identical to rover.py forward_kinematics(), adapted to Gazebo joint names.
        """
        # Corner angles (Gazebo frame: positive z up, so no sign flip needed vs rover.py)
        theta_fl = self.curr_positions.get('front_wheel_joint_L', 0.0)
        theta_fr = self.curr_positions.get('front_wheel_joint_R', 0.0)
        theta_bl = self.curr_positions.get('rear_wheel_joint_L', 0.0)
        theta_br = self.curr_positions.get('rear_wheel_joint_R', 0.0)

        def _radius_from_angle(angle):
            try:
                return self.d3 / math.tan(angle)
            except ZeroDivisionError:
                return float('inf')

        if theta_fl + theta_fr + theta_bl + theta_br > 0:  # turning left
            r_fc = self.d1  + _radius_from_angle(theta_fl)
            r_ff = -self.d1 + _radius_from_angle(theta_fr)
            r_bc = -self.d1 - _radius_from_angle(theta_bl)
            r_bf = self.d1  - _radius_from_angle(theta_br)
        else:  # turning right
            r_ff = self.d1  + _radius_from_angle(theta_fl)
            r_fc = -self.d1 + _radius_from_angle(theta_fr)
            r_bf = -self.d1 - _radius_from_angle(theta_bl)
            r_bc = self.d1  - _radius_from_angle(theta_br)

        approx_radius = sum(sorted([r_fc, r_ff, r_bc, r_bf])[1:3]) / 2.0
        if math.isnan(approx_radius):
            approx_radius = self.max_radius
        self.curr_turning_radius = approx_radius

        # Linear velocity from average of both middle wheels
        w_avg = (
            self.curr_velocities.get('middle_wheel_joint_left', 0.0)
            + self.curr_velocities.get('middle_wheel_joint_right', 0.0)
        ) / 2.0
        self.odometry.twist.twist.linear.x = w_avg * self.wheel_radius

        try:
            self.odometry.twist.twist.angular.z = (
                self.odometry.twist.twist.linear.x / self.curr_turning_radius
            )
        except ZeroDivisionError:
            self.odometry.twist.twist.linear.x = 0.0
            w_diff = (
                self.curr_velocities.get('middle_wheel_joint_left', 0.0)
                - self.curr_velocities.get('middle_wheel_joint_right', 0.0)
            ) / 2.0
            self.odometry.twist.twist.angular.z = w_diff * self.wheel_radius / self.d4

    # ------------------------------------------------------------------
    # Publishers
    # ------------------------------------------------------------------

    def _publish_wheel_velocities(self, vels: dict):
        msg = Float64MultiArray()
        msg.data = [vels.get(j, 0.0) for j in self.WHEEL_CMD_ORDER]
        self.wheel_pub.publish(msg)

    def _publish_servo_angles(self, angles: dict):
        msg = JointTrajectory()
        msg.joint_names = self.SERVO_CMD_ORDER
        pt = JointTrajectoryPoint()
        pt.positions  = [angles.get(j, 0.0) for j in self.SERVO_CMD_ORDER]
        pt.velocities = [0.0] * len(self.SERVO_CMD_ORDER)
        pt.time_from_start = Duration(sec=0, nanosec=200_000_000)  # 0.2 s
        msg.points.append(pt)
        self.servo_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RoverGazebo()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
