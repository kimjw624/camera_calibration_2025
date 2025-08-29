1. Camera service

- Install
sudo apt update
sudo apt install ros-humble-camera-info-manager
sudo apt install ros-humble-usb-cam

- Run camera (change args as needed)
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video0 -r image_raw:=/camera/image_raw

2. Camera calibration


ros2 run camera_calibration cameracalibrator --size <as-needed ex: 7x9> --square <as-needed ex: 0.02> --ros-args -r image:=/camera/image_raw -p camera:=/camera
