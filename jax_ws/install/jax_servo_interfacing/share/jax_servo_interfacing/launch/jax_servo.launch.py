from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='jax_servo_interfacing',
            executable='jax_servo_node',
            name='jax_servo_node',
            output='screen'
        )
    ])

