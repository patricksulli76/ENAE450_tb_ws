import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/patrick/turtlebot3_ENAE450_ws/install/turtlebot3_example'
