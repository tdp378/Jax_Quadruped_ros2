import math


class HardwareInterface:
    """
    Hardware abstraction layer for JAX servos.

    This class is intentionally dumb:
    - It accepts joint names and angles
    - It optionally forwards them to real hardware
    - It does NOT know about ROS, gaits, or trajectories
    """

    def __init__(self, use_hardware: bool = False):
        self.use_hardware = use_hardware

        # Joint → PCA9685 channel mapping
        # (placeholder — we will fill this in later)
        self.joint_to_channel = {}

        if self.use_hardware:
            self._init_hardware()
        else:
            self._init_sim()

    def _init_hardware(self):
        # Import here so sim mode never touches I2C
        from adafruit_servokit import ServoKit

        self.kit = ServoKit(channels=16)

        print("[HW] Servo hardware initialized")

    def _init_sim(self):
        self.kit = None
        print("[SIM] Hardware interface running in simulation mode")

    def set_joint_angle(self, joint_name: str, angle_deg: float):
        """
        Command a joint to a specific angle (degrees).
        """

        angle_deg = float(angle_deg)

        if self.use_hardware:
            if joint_name not in self.joint_to_channel:
                print(f"[HW WARNING] Unknown joint: {joint_name}")
                return

            channel = self.joint_to_channel[joint_name]

            # Safety clamp
            angle_deg = max(0.0, min(180.0, angle_deg))

            self.kit.servo[channel].angle = angle_deg
        else:
            print(f"[SIM] {joint_name} -> {angle_deg:.1f}°")

    def shutdown(self):
        """
        Called on node shutdown.
        """
        print("[HW] Shutting down hardware interface")
