import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kraken/kraken-nano/ROS/ws/src/install/kraken'
