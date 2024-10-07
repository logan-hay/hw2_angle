import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/mnt/c/Users/ljhay/hw2_angle/install/webots_ros2_homework1_python'
