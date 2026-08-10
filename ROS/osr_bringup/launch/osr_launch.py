import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    roboclaw_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'roboclaw_params.yaml'
    )
    osr_params = os.path.join(
        get_package_share_directory('osr_bringup'),
        'config',
        'osr_params.yaml'
    )
    
    node_roboclaw = Node(
        package='osr_control',
        executable='roboclaw_wrapper',
        name='roboclaw_wrapper',
        output='screen',
        emulate_tty=True,
        respawn=True,
        parameters=[roboclaw_params]
    )

    node_servo = Node(
        package='osr_control',
        executable='servo_control',
        name='servo_wrapper',
        output='screen',
        emulate_tty=True,
        respawn=True,
        parameters=[{'centered_pulse_widths': [147, 165, 160, 152]}]  # pulse width where the corner motors are in their default position, see rover_bringup.md.
        # indices du tableau :
        # 0 = back_right
        # 1 = front_right
        # 2 = front_left
        # 3 = back_left
    )

    arg_enable_odom = DeclareLaunchArgument('enable_odometry', default_value='false')
    
    arg_publish_tf = DeclareLaunchArgument('publish_transform', default_value='true')

    node_rover = Node(
        package='osr_control',
        executable='rover',
        name='rover',
        output='screen',
        emulate_tty=True,
        respawn=True,
        parameters=[
            osr_params,
            {'enable_odometry': LaunchConfiguration('enable_odometry'),
            'publish_transform': LaunchConfiguration('publish_transform')}
        ]
    )

    node_teleop = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        emulate_tty=True,
        respawn=True,
        parameters=[
            {"scale_linear.x": 1.3}, # scale to apply to drive speed, in m/s: drive_motor_rpm * 2pi / 60 * wheel radius * slowdown_factor
            {"axis_linear.x": 1}, # ! xbox=1 !
            {"axis_angular.yaw": 0},  # ! xbox=0 ! which joystick axis to use for driving
            {"axis_angular.pitch": 2},  # ! xbox=2 ! axis to use for in-place rotation
            {"scale_angular.yaw": 2.88},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
            {"scale_angular.pitch": 2.88},  # scale to apply to angular speed, in rad/s: scale_linear / min_radius(=0.45m)
            {"scale_angular_turbo.yaw": 3.88},  # scale to apply to angular speed, in rad/s: scale_linear_turbo / min_radius
            {"scale_linear_turbo.x": 1.75},  # scale to apply to linear speed, in m/s
            {"enable_button": 7},  # ! xbox=7 ! which button to press to enable movement
            {"enable_turbo_button": 6},  # ! xbox=6 ! -1 to disable turbo
            {"require_enable_button": True}
        ],
        remappings=[
            ('/cmd_vel', '/cmd_vel_intuitive')
        ]
    )

    node_joy = Node(
        package='joy',
        executable='joy_node',
        name='joy',
        output='screen',
        emulate_tty=True,
        respawn=True,
        parameters=[
            {"deadzone": 0.1},
            {"autorepeat_rate": 5.0},
            {"device_id": 0},  # ! default = 0 ! This might be different on your computer. Run `ls -l /dev/input/event*`. If you have event1, put 1.
        ]        
    )

    node_ina = Node(
        package='osr_control',
        executable='ina260',
        name='ina260_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            {"publish_rate": 1.0},
            {"sensor_address": "0x45"},
        ]        
    )

    # ld.add_action(
    #     Node(
    #         package='osr_control',
    #         executable='joy_extras',
    #         output='screen',
    #         emulate_tty=True,
    #         parameters=[
    #             {"duty_button_index": 1}  # which button toggles duty mode on/off
    #         ]
    #     )
    # )

    node_lidar = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        output='screen',
        parameters=[
            {'channel_type': 'serial'},
            {'serial_port': '/dev/ttyUSB0'},
            {'serial_baudrate': 115200},
            {'frame_id': 'laser'},
            {'inverted': False},
            {'angle_compensate': True}
        ]
    )

    node_rf2o = Node(
        package='rf2o_laser_odometry',
        executable='rf2o_laser_odometry_node',
        name='rf2o_laser_odometry',
        output='screen',
        parameters=[{
            'laser_scan_topic' : '/scan',
            'odom_topic' : '/odom_rf2o',
            'publish_tf' : True,
            'base_frame_id' : 'base_footprint',
            'odom_frame_id' : 'odom',
            'init_pose_from_topic' : '',
            'freq' : 10.0}],
    )

    return LaunchDescription([
        arg_enable_odom,
        arg_publish_tf,
        node_roboclaw,
        node_servo,
        node_rover,
        node_teleop,
        node_joy,
        node_ina,
        TimerAction(period=1.0, actions=[node_lidar]),
        TimerAction(period=3.0, actions=[node_rf2o]),
    ])
