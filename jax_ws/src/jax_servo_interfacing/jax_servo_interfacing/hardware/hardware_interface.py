from typing import Optional 

import numpy as np

# Try to import hardware libraries safely
try:
    from adafruit_servokit import ServoKit
    HARDWARE_AVAILABLE = True
except ImportError:
    ServoKit = None
    HARDWARE_AVAILABLE = False


class HardwareInterface:
    """
    Low-level hardware interface for servo control.

    This class ONLY talks to hardware.
    - No ROS
    - No leg logic
    - No joint names
    """

    def __init__(self, link: Optional[str] = None, enable_hardware: bool = True):
        """
        :param link: Placeholder for future transport abstraction (unused for now)
        :param enable_hardware: If False, runs in simulation / dry-run mode
        """
        self.link = link
        self.enable_hardware = enable_hardware and HARDWARE_AVAILABLE

        self.num_channels = 16
        self.kit = None

        if self.enable_hardware:
            try:
                self.kit = ServoKit(channels=self.num_channels)
                print("[HardwareInterface] ServoKit initialized")
            except Exception as e:
                print(f"[HardwareInterface] Failed to initialize hardware: {e}")
                print("[HardwareInterface] Falling back to no-hardware mode")
                self.kit = None
                self.enable_hardware = False
        else:
            print("[HardwareInterface] Running in no-hardware mode")

    # -------------------------
    # Core servo operations
    # -------------------------

    def set_servo_angle(self, channel: int, angle: float):
        """
        Set a servo angle on a given PCA9685 channel.

        :param channel: PCA9685 channel (0–15)
        :param angle: Servo angle in degrees
        """
        if not 0 <= channel < self.num_channels:
            raise ValueError(f"Invalid servo channel: {channel}")

        if self.enable_hardware and self.kit:
            self.kit.servo[channel].angle = angle
        else:
            print(f"[HardwareInterface] (SIM) Channel {channel} -> {angle:.2f}°")

    def disable_servo(self, channel: int):
        """
        Disable a servo output.
        """
        if not 0 <= channel < self.num_channels:
            raise ValueError(f"Invalid servo channel: {channel}")

        if self.enable_hardware and self.kit:
            self.kit.servo[channel].angle = None
        else:
            print(f"[HardwareInterface] (SIM) Channel {channel} disabled")

    # -------------------------
    # Batch joint control
    # -------------------------

    def set_multiple_joints(self, joint_angles: np.ndarray):
        """
        Set multiple joint angles at once.

        Expected input:
            joint_angles: numpy array of shape (3, 4)
                - 3 joints per leg
                - 4 legs

        Temporary channel mapping:
            channel = joint_index * 4 + leg_index
        """

        if not isinstance(joint_angles, np.ndarray):
            raise TypeError("joint_angles must be a numpy array")

        if joint_angles.shape != (3, 4):
            raise ValueError(
                f"Expected joint_angles shape (3, 4), got {joint_angles.shape}"
            )

        for joint_idx in range(3):
            for leg_idx in range(4):
                channel = joint_idx * 4 + leg_idx
                angle_rad = joint_angles[joint_idx, leg_idx]

                # Convert radians → degrees for servos
                angle_deg = np.degrees(angle_rad)

                self.set_servo_angle(channel, angle_deg)

    # -------------------------
    # Shutdown
    # -------------------------

    def shutdown(self):
        """
        Safely disable all servos.
        """
        print("[HardwareInterface] Shutting down servos")

        for ch in range(self.num_channels):
            try:
                self.disable_servo(ch)
            except Exception:
                pass
