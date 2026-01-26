import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kunal-singh/Desktop/Self_Moving_Car/install/circular_drive_controller'
