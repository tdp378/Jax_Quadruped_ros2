from jax_servo_interfacing.joint_names
import JOINT_NAME_MATRIX

self.joint_name_to_index = {}

for axis in range(3):
  for leg in range(4):
    joint_name = JOINT_NAME_MATRIX[axis][leg]

self.joint_name_to_index[joint_name] = (axis, leg)
