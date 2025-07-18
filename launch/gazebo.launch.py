from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():

    pkg_share = FindPackageShare('ackermann_pipe').find('ackermann_pipe')
    urdf_path = os.path.join(pkg_share, 'urdf', 'robot.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz', 'config_lidar.rviz')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),

        ExecuteProcess(
            cmd=[
                'gazebo',
                '--verbose',
                '-s', 'libgazebo_ros_init.so',
                '-s', 'libgazebo_ros_factory.so'
            ],
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': Command(['xacro ', urdf_path])
            }]
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-entity', 'robot', '-topic', 'robot_description'],
            output='screen'
        ),

        ExecuteProcess(
            cmd=['rviz2', '-d', rviz_config],
            output='screen'
        )
    ])
