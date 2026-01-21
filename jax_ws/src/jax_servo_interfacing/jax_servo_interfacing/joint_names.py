LEG_NAMES = [ 
    "front_right",
    "front_left",
    "rear_right",
    "rear_left",
]

JOINT_NAMES = [
    "hip",
    "thigh",
    "calf",
]

# Maps (axis, leg) → joint_name
JOINT_NAME_MATRIX = [
    [f"{leg}_{joint}_joint" for leg in LEG_NAMES]
    for joint in JOINT_NAMES
]

# Flattened version (useful later)
JOINT_NAMES_FLAT = [
    name
    for row in JOINT_NAME_MATRIX
    for name in row
]

