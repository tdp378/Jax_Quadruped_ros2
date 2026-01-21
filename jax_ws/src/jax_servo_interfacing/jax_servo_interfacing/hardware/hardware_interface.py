#!/usr/bin/env python3 

from adafruit_servokit import ServoKit
import numpy as np
import math as m

from jax_servo_interfacing.joint_names import JOINT_NAME_MATRIX


class HardwareInterface:
    """
    Low-level hardware interface for JAX quadruped servos.

    This class:
    - Converts IK joint angles to servo angles
    - Applies calibration, limits, and linkage math
    - Maps ROS-style joint names to physical servos
    """

    def __init__(self, link):
        # PWM limits
        self.pwm_max = 2400
        self.pwm_min = 370

        # Mechanical linkage model
        self.link = link

        # Servo state (degrees)
        self.servo_angles = np.zeros((3, 4))

        # PCA9685 servo driver
        self.kit = ServoKit(channels=16)

        # Servo pin mapping [axis, leg]
        self.pins = np.array([
            [14, 10, 2, 6], # hip
            [13, 9, 1, 5], # thigh
            [12, 8, 0, 4], # calf
        ])

        # Servo direction correction
        self.servo_multipliers = np.array([
            [-1, 1, 1, -1],
            [ 1, -1, 1, -1],
            [ 1, -1, 1, -1],
        ])

        self.complementary_angle = np.array([
            [180, 0, 0, 180],
            [0, 180, 0, 180],
            [0, 180, 0, 180],
        ])

        # Physical calibration offsets (degrees)
        self.physical_calibration_offsets = np.array([
            [75, 130, 113, 73],
            [29, 13, 33, 15],
            [26, 12, 30, 4],
        ])

        # -------- Joint name → (axis, leg) mapping --------
        self.joint_name_to_index = {}
        for axis in range(3):
            for leg in range(4):
                joint_name = JOINT_NAME_MATRIX[axis][leg]
                self.joint_name_to_index[joint_name] = (axis, leg)

        # Initialize servos
        self._initialize_servos()

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _initialize_servos(self):
        """Configure PWM ranges for all servos."""
        for i in range(16):
            self.kit.servo[i].actuation_range = 180
            self.kit.servo[i].set_pulse_width_range(
                self.pwm_min,
                self.pwm_max
            )

    # ------------------------------------------------------------------
    # Public command APIs
    # ------------------------------------------------------------------

    def set_actuator_positions(self, joint_angles):
        """
        Command all joints using a 3x4 array of radians.
        """
        possible_joint_angles = impose_physical_limits(joint_angles)
        self._joint_angles_to_servo_angles(possible_joint_angles)

        for leg in range(4):
            for axis in range(3):
                try:
                    pin = self.pins[axis, leg]
                    self.kit.servo[pin].angle = self.servo_angles[axis, leg]
                except Exception:
                    print("Warning: I2C servo write failed")

    def set_joint_angle(self, joint_name: str, angle_rad: float):
        """
        Set a single joint by name (radians).
        """
        axis, leg = self.joint_name_to_index[joint_name]
        self.servo_angles[axis, leg] = m.degrees(angle_rad)

    def set_joint_angles_by_name(self, joint_angle_dict):
        """
        Set multiple joints by name.

        Example:
        {
            "front_left_hip_joint": 0.2,
            "front_left_thigh_joint": 0.6
        }
        """
        for name, angle_rad in joint_angle_dict.items():
            axis, leg = self.joint_name_to_index[name]
            self.servo_angles[axis, leg] = m.degrees(angle_rad)

    def relax_all_motors(self, servo_mask=np.ones((3, 4))):
        """
        Disable selected servos.
        """
        for leg in range(4):
            for axis in range(3):
                if servo_mask[axis, leg]:
                    pin = self.pins[axis, leg]
                    self.kit.servo[pin].angle = None

    # ------------------------------------------------------------------
    # Internal conversions
    # ------------------------------------------------------------------

    def _joint_angles_to_servo_angles(self, joint_angles):
        """
        Convert IK joint angles to calibrated servo angles.
        """
        for leg in range(4):
            hip, thigh, calf = joint_angles[:, leg]

            theta0 = lower_leg_angle_to_servo_angle(
                self.link,
                m.pi / 2 - thigh,
                calf + m.pi / 2
            )

            self.servo_angles[0, leg] = m.degrees(hip)
            self.servo_angles[1, leg] = m.degrees(thigh)
            self.servo_angles[2, leg] = m.degrees(m.pi / 2 + m.pi - theta0)

        self.servo_angles = np.clip(
            self.servo_angles + self.physical_calibration_offsets,
            0,
            180
        )

        self.servo_angles = np.round(
            self.servo_angles * self.servo_multipliers
            + self.complementary_angle,
            1
        )


# ======================================================================
# Helper functions (unchanged math)
# ======================================================================

def calculate_4_bar(th2, a, b, c, d):
    x_b = a * np.cos(th2)
    y_b = a * np.sin(th2)

    f = np.sqrt((d - x_b) ** 2 + y_b ** 2)
    beta = np.arccos((f ** 2 + c ** 2 - b ** 2) / (2 * f * c))
    gamma = np.arctan2(y_b, d - x_b)

    th4 = np.pi - gamma - beta
    x_c = c * np.cos(th4) + d
    y_c = c * np.sin(th4)

    th3 = np.arctan2(y_c - y_b, x_c - x_b)

    ABC = np.pi - th2 + th3
    BCD = th4 - th3
    CDA = 2 * np.pi - th2 - ABC - BCD

    return ABC, BCD, CDA


def lower_leg_angle_to_servo_angle(link, THETA2, THETA3):
    GDE, _, _ = calculate_4_bar(
        THETA3 + link.lower_leg_bend_angle,
        link.i, link.h, link.f, link.g
    )

    CDH = 1.5 * m.pi - THETA2 - GDE - link.EDC
    CDA = CDH + link.gamma

    DAB, _, _ = calculate_4_bar(
        CDA, link.d, link.a, link.b, link.c
    )

    return DAB + link.gamma


def impose_physical_limits(desired_joint_angles):
    possible = np.zeros((3, 4))

    for i in range(4):
        hip, upper, lower = np.degrees(desired_joint_angles[:, i])

        hip = np.clip(hip, -20, 20)
        upper = np.clip(upper, 0, 120)

        if upper < 10:
            lower = np.clip(lower, -20, 40)
        elif upper < 20:
            lower = np.clip(lower, -40, 40)
        elif upper < 30:
            lower = np.clip(lower, -50, 40)
        elif upper < 40:
            lower = np.clip(lower, -60, 30)
        elif upper < 50:
            lower = np.clip(lower, -70, 25)
        elif upper < 60:
            lower = np.clip(lower, -70, 20)
        elif upper < 70:
            lower = np.clip(lower, -70, 0)
        elif upper < 80:
            lower = np.clip(lower, -70, -10)
        elif upper < 90:
            lower = np.clip(lower, -70, -20)
        elif upper < 100:
            lower = np.clip(lower, -70, -30)
        elif upper < 110:
            lower = np.clip(lower, -70, -40)
        else:
            lower = np.clip(lower, -70, -60)

        possible[:, i] = hip, upper, lower

    return np.radians(possible)
