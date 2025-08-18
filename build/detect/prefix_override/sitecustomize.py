import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/wanglonglong/Learn_ROS2/03.my_robot/install/detect'
