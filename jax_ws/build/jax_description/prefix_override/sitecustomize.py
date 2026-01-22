import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/tdp378/Jax_Quadruped_ros2/jax_ws/install/jax_description'
