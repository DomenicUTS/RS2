import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/domenic/RS2/ros2_ws/install/ur3_motion_planning'
