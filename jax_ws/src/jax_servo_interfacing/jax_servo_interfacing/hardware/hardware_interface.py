from typing import Dict, Optional

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
        """
        if not 0 <= channel < self.num_channels:
            raise ValueError(f"Invalid servo channel: {channel}")

        if self.enable_hardware and self.kit:
            self.kit.servo[channel].angle = angle
        else:
            print(f"[HardwareInterface] (SIM) Channel {channel} -> {angle:.2f}°")

    def set_multiple_joints(self, channel_angle_map: Dict[int, float]):
        """
        Set multiple servos at once.

        :param channel_angle_map: {channel: angle_degrees}
        """
        for channel, angle in channel_angle_map.items():
            self.set_servo_angle(channel, angle)

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
