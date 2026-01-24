from math import copysign
from .config.jax_servo_config import (
    JOINT_TO_SERVO,
    NEUTRAL_RAD,
    DIRECTION,
    JOINT_LIMITS,
)

def clamp(value, limits):
    return max(limits[0], min(limits[1], value))


def joint_type(joint_name):
    if "hip" in joint_name:
        return "hip"
    if "thigh" in joint_name:
        return "thigh"
    if "knee" in joint_name:
        return "knee"
    raise ValueError(f"Unknown joint type: {joint_name}")


def joints_to_servo_commands(joint_positions):
    """
    joint_positions: dict[str, float]  (radians)

    returns: dict[int, float]
        servo_channel -> commanded angle (rad)
    """

    servo_commands = {}

    for joint, angle in joint_positions.items():
        if joint not in JOINT_TO_SERVO:
            continue

        jtype = joint_type(joint)
        limits = JOINT_LIMITS[jtype]

        # Clamp commanded angle
        angle = clamp(angle, limits)

        # Apply direction + neutral
        servo_angle = (
            NEUTRAL_RAD[joint]
            + DIRECTION[joint] * angle
        )

        channel = JOINT_TO_SERVO[joint]
        servo_commands[channel] = servo_angle

    return servo_commands
