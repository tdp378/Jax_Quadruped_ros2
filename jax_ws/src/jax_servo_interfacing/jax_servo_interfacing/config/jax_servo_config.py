# jax_servo_interfacing/config/jax_servo_config.py 

# Canonical joint order
JOINT_NAMES = [
    "front_left_hip_joint",
    "front_left_thigh_joint",
    "front_left_knee_joint",

    "front_right_hip_joint",
    "front_right_thigh_joint",
    "front_right_knee_joint",

    "rear_left_hip_joint",
    "rear_left_thigh_joint",
    "rear_left_knee_joint",

    "rear_right_hip_joint",
    "rear_right_thigh_joint",
    "rear_right_knee_joint",
]

# Joint → servo channel
JOINT_TO_SERVO = {
    "front_left_hip_joint": 0,
    "front_left_thigh_joint": 1,
    "front_left_knee_joint": 2,

    "front_right_hip_joint": 3,
    "front_right_thigh_joint": 4,
    "front_right_knee_joint": 5,

    "rear_left_hip_joint": 6,
    "rear_left_thigh_joint": 7,
    "rear_left_knee_joint": 8,

    "rear_right_hip_joint": 9,
    "rear_right_thigh_joint": 10,
    "rear_right_knee_joint": 11,
}

# Neutral offsets (radians)
NEUTRAL_RAD = {name: 0.0 for name in JOINT_NAMES}

# Direction multipliers
DIRECTION = {
    "front_left_hip_joint": +1,
    "front_left_thigh_joint": +1,
    "front_left_knee_joint": +1,

    "front_right_hip_joint": -1,
    "front_right_thigh_joint": +1,
    "front_right_knee_joint": +1,

    "rear_left_hip_joint": +1,
    "rear_left_thigh_joint": +1,
    "rear_left_knee_joint": +1,

    "rear_right_hip_joint": -1,
    "rear_right_thigh_joint": +1,
    "rear_right_knee_joint": +1,
}

# Joint limits (radians)
JOINT_LIMITS = {
    "hip": (-0.6, 0.6),
    "thigh": (-1.0, 1.0),
    "knee": (0.0, 2.0),
}

