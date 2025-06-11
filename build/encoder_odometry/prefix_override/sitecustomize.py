import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/pyo/Workspace/turtlebot_ws/install/encoder_odometry'
