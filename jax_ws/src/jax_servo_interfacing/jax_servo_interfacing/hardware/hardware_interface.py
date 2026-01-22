from typing import Optional, Dict 

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
    - No gait logic
    - No IK
    """

    def __init__(self, link: Optional[str] = None, enable_hardware: bool = True):
        self.link = link
        self.enable_hardware = enable_hardware and HARDWARE_AVAILABLE

        self.num_channels = 16
        self.kit = None

        # 🔹 TEMP joint → channel map
        # (we will formalize this later)
        self.joint_channel_map = {
            "front_left_hip": 0,
            "front_left_knee": 1,
            "front_right_hip": 2,
            "front_right_knee": 3,
            "rear_left_hip": 4,
            "rear_left_knee": 5,
            "rear_right_hip": 6,
            "rear_right_knee": 7,
        }

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
        if not 0 <= channel < self.num_channels:
            raise ValueError(f"Invalid servo channel: {channel}")

        if self.enable_hardware and self.kit:
            self.kit.servo[channel].angle = angle
        else:
            print(f"[HardwareInterface] (SIM) Channel {channel} -> {angle:.2f}°")

    def disable_servo(self, channel: int):
        if not 0 <= channel < self.num_channels:
            raise ValueError(f"Invalid servo channel: {channel}")

        if self.enable_hardware and self.kit:
            self.kit.servo[channel].angle = None
        else:
            print(f"[HardwareInterface] (SIM) Channel {channel} disabled")

    # -------------------------
    # Joint-level API (THIS fixes your error)
    # -------------------------

    def set_joint_angle(self, joint_name: str, angle: float):
        if joint_name not in self.joint_channel_map:
            raise KeyError(f"Unknown joint: {joint_name}")

        channel = self.joint_channel_map[joint_name]
        self.set_servo_angle(channel, angle)

    def set_multiple_joints(self, joint_angles: Dict[str, float]):
        """
        joint_angles: { joint_name: angle }
        """
        for joint, angle in joint_angles.items():
            self.set_joint_angle(joint, angle)

    # -------------------------
    # Shutdown
    # -------------------------

    def shutdown(self):
        print("[HardwareInterface] Shutting down servos")
        for ch in range(self.num_channels):
            try:
                self.disable_servo(ch)
            except Exception:
                pass
