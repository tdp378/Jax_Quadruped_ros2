#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np

# Import hardware layer
from jax_servo_interfacing.hardware.hardware_interface import JaxHardwareInterface


class JaxServoNode(Node):
    def __init__(self):
        super().__init__('jax_servo_node')

        # Parameters
        self.declare_parameter('dry_run', True)
        self.declare_parameter('update_rate', 50.0)

        self.dry_run = self.get_parameter('dry_run').value

        # Subscriber
        self.subscription = self.create_subscription(
            JointState,
            '/jax/joint_commands',
            self.joint_command_callback,
            10
        )

        self.get_logger().info(
            f'Jax Servo Node started (dry_run={self.dry_run})'
        )

        # Hardware interface (only if not dry-run)
        self.hardware = None
        if not self.dry_run:
            self.get_logger().info('Initializing hardware interface...')
            self.hardware = JaxHardwareInterface(link=None)  # link passed later

    def joint_command_callback(self, msg: JointState):
        """
        Expects joint angles in radians.
        Order: [hip, shoulder, knee] for each leg.
        """
        if len(msg.position) != 12:
            self.get_logger().error(
                f'Expected 12 joint angles, got {len(msg.position)}'
            )
            return

        # Convert to 3x4 numpy array
        joint_angles = np.array(msg.position).reshape((4, 3)).T

        if self.dry_run:
            self.get_logger().info(
                f'[DRY RUN] Joint angles:\n{joint_angles}'
            )
            return

        try:
            self.hardware.set_actuator_positions(joint_angles)
        except Exception as e:
            self.get_logger().error(f'Hardware error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = JaxServoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

