from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_path = get_package_share_directory('jax_description')
    urdf_path = os.path.join(pkg_path, 'urdf', 'jax.urdf.xacro')
    rviz_path = os.path.join(pkg_path, 'rviz', 'jax.rviz')

    return LaunchDescription([

        DeclareLaunchArgument(
            'use_gui',
            default_value='true',
            description='Use joint_state_publisher_gui'
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
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=None
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_path],
            output='screen'
        ),
    ])
