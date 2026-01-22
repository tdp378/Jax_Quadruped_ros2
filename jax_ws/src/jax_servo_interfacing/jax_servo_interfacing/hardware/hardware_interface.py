# jax_servo_interfacing/hardware/hardware_interface.py

from typing import Dict

try:
    from adafruit_servokit import ServoKit
    HARDWARE_LIB_AVAILABLE = True
except ImportError as e:
    ServoKit = None
    HARDWARE_LIB_AVAILABLE = False
    print(f"[WARN] Adafruit ServoKit not available: {e}")


class HardwareInterface:
    """
    Low-level hardware interface for the PCA9685 servo controller.
    This class MUST NOT crash if hardware is missing.
    """

    def __init__(self, link=None, i2c_address: int = 0x40, channels: int = 16):
        self.link = link
        self.i2c_address = i2c_address
        self.channels = channels

        self.hardware_ok = False
        self.kit = None

        self._initialize_hardware()

    def _initialize_hardware(self):
        """Attempt to initialize servo hardware safely."""
        if not HARDWARE_LIB_AVAILABLE:
            print("[WARN] ServoKit library not available. Running in no-hardware mode.")
            return

        try:
            self.kit = ServoKit(
                channels=self.channels,
                address=self.i2c_address
            )
            self.hardware_ok = True
            print("[INFO] PCA9685 servo controller initialized successfully")

        except Exception as e:
            self.kit = None
            self.hardware_ok = False
            print(f"[WARN] PCA9685 not detected at 0x{self.i2c_address:02X}: {e}")
            print("[WARN] Running in no-hardware mode")

    def set_servo_angle(self, channel: int, angle: float):
        """
        Set a servo angle safely.
        """
        if not self.hardware_ok:
            return

        if channel < 0 or channel >= self.channels:
            print(f"[WARN] Invalid servo channel: {channel}")
            return

        try:
            self.kit.servo[channel].angle = angle
        except Exception as e:
            print(f"[ERROR] Failed to set servo {channel}: {e}")

    def shutdown(self):
        """
        Clean shutdown hook.
        """
        if self.hardware_ok:
            print("[INFO] Shutting down servo hardware")
