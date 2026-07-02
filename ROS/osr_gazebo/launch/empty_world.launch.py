import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    package_name = 'osr_gazebo'

    default_world = os.path.join(get_package_share_directory(package_name), 'worlds', 'island.sdf')

    world = LaunchConfiguration('world')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                    launch_arguments={
                        'gz_args': ['-r -v4 ', world],
                        'on_exit_shutdown': 'true'
                    }.items()
             )

    xacro_file = os.path.join(get_package_share_directory(package_name), 'urdf', 'osr.urdf.xacro')
    robot_description_urdf = xacro.process_file(xacro_file).toxml()

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_urdf,
            'use_sim_time': True
        }]
    )

    node_controller_spawn = Node(
        package=package_name,
        executable='osr_controller',
        output='screen'
    )
    
    node_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'rover',
            '-z', '0.8',            # Pour surelever si il se bug dans le sol
        ],
        output='screen'
    )

    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    node_ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{
            'config_file': bridge_params
        }]
    )

    node_ros_gz_image = Node(
        package='ros_gz_image',
        executable='image_bridge',
        output='screen',
        arguments=['/camera/image_raw']
    )

    # joint_state_controller
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
        output='screen'
    )

    # wheel_velocity_controller
    rover_wheel_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'wheel_controller'],
        output='screen'
    )

    # servo_controller
    servo_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'servo_controller'],
        output='screen'
    )

    # Launch RViz
    rviz_config_file = os.path.join(get_package_share_directory(package_name), 'rviz/rviz_settings2.rviz')
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )
    
    return LaunchDescription([
    	node_controller_spawn,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=node_spawn_entity,
                on_exit=[
                    load_joint_state_controller,
                    rover_wheel_controller,
                    servo_controller,
                ],
            )
        ),
   
        world_arg,
        gazebo,
        node_robot_state_publisher,
        node_spawn_entity,
        node_ros_gz_bridge,
        node_rviz,
        node_ros_gz_image,
    ])
