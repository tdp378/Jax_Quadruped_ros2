#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node

import numpy as np

# Import your hardware layer
from jax_servo_interfacing.hardware.hardware_interface import HardwareInterface


class JaxServoNode(Node):
    """
    ROS 2 node responsible for commanding the JAX quadruped servos.
    """

    def __init__(self):
        super().__init__('jax_servo_node')

        self.get_logger().info('Starting JAX Servo Node')

        # -----------------------------
        # Hardware interface
        # -----------------------------
        # NOTE: link is None for now.
        # Later this will be replaced with your linkage/config object.
        self.hardware = HardwareInterface(link=None)

        # -----------------------------
        # Temporary test timer
        # -----------------------------
        # This lets us test the node WITHOUT ROS topics or hardware
        self.timer = self.create_timer(2.0, self.test_motion)

        self.get_logger().info('JAX Servo Node initialized')

    def test_motion(self):
        """
        Temporary test function.
        Sends a neutral pose to the hardware interface.
        This will be removed once we add real subscriptions.
        """

        self.get_logger().info('Sending test joint angles')

        # 3 joints x 4 legs (radians)
        joint_angles = np.zeros((3, 4))

        try:
            self.hardware.set_actuator_postions(joint_angles)
        except Exception as e:
            self.get_logger().warn(f'Hardware call failed: {e}')


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
