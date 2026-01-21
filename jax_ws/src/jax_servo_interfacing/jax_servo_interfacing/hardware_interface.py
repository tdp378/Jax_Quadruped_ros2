#!/usr/bin/env python3 
"""
Pure hardware interface for the Jax quadruped.

This module is ROS-agnostic and should be called by a ROS 2 node.
It handles servo calibration, linkage math, and low-level actuation.
"""

from adafruit_servokit import ServoKit
import numpy as np
import math as m


class JaxHardwareInterface:
    def __init__(self, link):
        self.pwm_max = 2400
        self.pwm_min = 370
        self.link = link
        self.servo_angles = np.zeros((3, 4))
        self.kit = ServoKit(channels=16)

        # Servo board pin mapping
        self.pins = np.array([
            [14, 10, 2, 6],
            [13, 9, 1, 5],
            [12, 8, 0, 4]
        ])

        # Angle direction corrections
        self.servo_multipliers = np.array([
            [-1, 1, 1, -1],
            [ 1, -1, 1, -1],
            [ 1, -1, 1, -1]
        ])

        self.complementary_angle = np.array([
            [180, 0, 0, 180],
            [0, 180, 0, 180],
            [0, 180, 0, 180]
        ])

        # Physical calibration offsets (degrees)
        self.physical_calibration_offsets = np.array([
            [75, 130, 113, 73],
            [29, 13, 33, 15],
            [26, 12, 30, 4]
        ])

        self._initialize_servos()

    def _initialize_servos(self):
        for i in range(16):
            self.kit.servo[i].actuation_range = 180
            self.kit.servo[i].set_pulse_width_range(
                self.pwm_min,
                self.pwm_max
            )

    def set_actuator_positions(self, joint_angles):
        """
        Convert joint angles (radians) to servo commands and actuate.
        """
        possible_joint_angles = impose_physical_limits(joint_angles)
        self._joint_angles_to_servo_angles(possible_joint_angles)

        for leg_index in range(4):
            for axis_index in range(3):
                try:
                    self.kit.servo[
                        self.pins[axis_index, leg_index]
                    ].angle = self.servo_angles[axis_index, leg_index]
                except Exception:
                    print("[JaxHardwareInterface] Warning - I2C IO error")

    def relax_all_motors(self, servo_list=np.ones((3, 4))):
        """
        Disable selected servos.
        """
        for leg_index in range(4):
            for axis_index in range(3):
                if servo_list[axis_index, leg_index] == 1:
                    self.kit.servo[
                        self.pins[axis_index, leg_index]
                    ].angle = None

    def _joint_angles_to_servo_angles(self, joint_angles):
        for leg in range(4):
            THETA2, THETA3 = joint_angles[1:, leg]

            THETA0 = lower_leg_angle_to_servo_angle(
                self.link,
                m.pi / 2 - THETA2,
                THETA3 + m.pi / 2
            )

            self.servo_angles[0, leg] = m.degrees(joint_angles[0, leg])
            self.servo_angles[1, leg] = m.degrees(THETA2)
            self.servo_angles[2, leg] = m.degrees(m.pi / 2 + m.pi - THETA0)

        self.servo_angles = np.clip(
            self.servo_angles + self.physical_calibration_offsets,
            0,
            180
        )

        self.servo_angles = np.round(
            np.multiply(self.servo_angles, self.servo_multipliers)
            + self.complementary_angle,
            1
        )
