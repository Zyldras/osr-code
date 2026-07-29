import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    node_teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[
            {"scale_linear.x": 1.3}, # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor
            {"axis_linear.x": 1}, # ! xbox=1 !
            {"axis_angular.yaw": 0},  # ! xbox=0 ! which joystick axis to use for driving
            {"axis_angular.pitch": 2},  # ! xbox=2 ! axis to use for in-place rotation
            {"scale_angular.yaw": 2.88},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
            {"scale_angular.pitch": 2.88},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
            {"scale_angular_turbo.yaw": 3.88},  # scale to apply to angular speed, in rad/s: scale_linear_turbo / min_radius
            {"scale_linear_turbo.x": 1.75},  # scale to apply to linear speed, in m/s
            {"enable_button": 10},  # ! xbox=7 ! which button to press to enable movement
            {"enable_turbo_button": 9},  # ! xbox=6 ! -1 to disable turbo
            {"require_enable_button": True}
        ],
        # remappings=[
        #     ('/cmd_vel', '/cmd_vel_intuitive')
        # ]
    )

    node_joy = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        parameters=[
            {"deadzone": 0.05},
            {"autorepeat_rate": 5.0},
            {"device_id": 0},  # ! default = 0 ! This might be different on your computer. Run `ls -l /dev/input/event*`. If you have event1, put 1.
        ]
    )
    
    return LaunchDescription([
        node_joy,
        node_teleop
    ])