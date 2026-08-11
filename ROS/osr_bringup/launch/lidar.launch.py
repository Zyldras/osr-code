import os

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():

    node_lidar = Node(
        package="rplidar_ros",
        executable="rplidar_node",
        name="rplidar_node",
        output="screen",
        parameters=[
            {
                "channel_type": "serial",
                "serial_port": "/dev/ttyUSB0",
                "serial_baudrate": 115200,
                "frame_id": "laser",
                "inverted": False,
                "angle_compensate": True,
                "scan_mode": "Standard",
                "scan_frequency": 5.0,
                "use_sim_time": False,
            }
        ],
    )

    node_rf2o = Node(
        package="rf2o_laser_odometry",
        executable="rf2o_laser_odometry_node",
        name="rf2o_laser_odometry",
        output="screen",
        parameters=[
            {
                "laser_scan_topic": "/scan",
                "odom_topic": "/odom_rf2o",
                "publish_tf": True,
                "base_frame_id": "base_footprint",
                "odom_frame_id": "odom",
                "init_pose_from_topic": "",
                "freq": 5.0,
            }
        ],
    )

    return LaunchDescription(
        [
            TimerAction(period=1.0, actions=[node_lidar]),
            TimerAction(period=5.0, actions=[node_rf2o]),
        ]
    )
