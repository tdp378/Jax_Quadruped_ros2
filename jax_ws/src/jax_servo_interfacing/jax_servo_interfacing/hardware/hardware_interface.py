# jax_servo_interfacing/hardware/hardware_interface.py

from typing import Optional

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
    - No joint names
    - No leg logic
    - No ROS
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
            # Dry-run output for debugging / simulation
            print(f"[HardwareInterface] (SIM) Channel {channel} -> {angle:.2f}°")

    def disable_servo(self, channel: int):
        """
        Disable a servo output (sets pulse to None).
        """
        if not 0 <= channel < self.num_channels:
            raise ValueError(f"Invalid servo channel: {channel}")

        if self.enable_hardware and self.kit:
            self.kit.servo[channel].angle = None
        else:
            print(f"[HardwareInterface] (SIM) Channel {channel} disabled")

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
