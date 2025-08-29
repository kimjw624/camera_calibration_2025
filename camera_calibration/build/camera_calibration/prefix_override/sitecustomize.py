import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jungwon/Desktop/camera_calibration/install/camera_calibration'
