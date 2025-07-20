import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ubuntu/Documents/uvic/kraken-nano/ROS/ws/src/install/kraken'
