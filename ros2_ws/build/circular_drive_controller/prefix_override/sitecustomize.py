import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kunal-singh/Desktop/Self-Moving-Car/ros2_ws/install/circular_drive_controller'
