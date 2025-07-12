import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/orin_nano1/humanoid/ros2_workspace/install/hardware_data'
