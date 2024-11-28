import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/px4space/ISR/Thesis/Code/ros2_ws_px4/install/gz_bridge'
