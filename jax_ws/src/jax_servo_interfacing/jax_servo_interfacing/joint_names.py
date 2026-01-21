LEG_NAMES = [
    "front_right",
    "front_left",
    "rear_right",
    "rear_left",
]

JOINT_NAMES = ["hip", "thigh", "calf"]

JOINT_NAMES_FULL = [
    f"{leg}_{joint}_joint"
    for leg in LEG_NAMES
    for joint in JOINT_NAMES
]

