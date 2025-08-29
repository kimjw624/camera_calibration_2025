#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from sensor_msgs.srv import SetCameraInfo
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import yaml
import os
from pathlib import Path
import copy

def load_camera_info_from_yaml(yaml_path: str) -> CameraInfo:
    """Load camera info YAML produced by calibration tools and return a CameraInfo msg."""
    if not os.path.exists(yaml_path):
        raise FileNotFoundError(f"Camera info file not found: {yaml_path}")
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    ci = CameraInfo()
    ci.width = int(data.get('image_width', 0))
    ci.height = int(data.get('image_height', 0))
    ci.k = list(data.get('camera_matrix', {}).get('data', []))
    ci.d = list(data.get('distortion_coefficients', {}).get('data', []))
    ci.r = list(data.get('rectification_matrix', {}).get('data', []))
    ci.p = list(data.get('projection_matrix', {}).get('data', []))
    ci.distortion_model = data.get('distortion_model', 'plumb_bob')
    ci.header.frame_id = data.get('camera_name', '')
    return ci

def save_camera_info_to_yaml(ci: CameraInfo, yaml_path: str, camera_name: str = "") -> str:
    """Save CameraInfo msg to calibration-style YAML (compatible with camera_info_manager / calibrator)."""
    data = {
        'camera_name': camera_name or ci.header.frame_id or 'camera',
        'image_width': int(ci.width),
        'image_height': int(ci.height),
        'camera_matrix': {'rows': 3, 'cols': 3, 'data': list(ci.k)},
        'distortion_model': getattr(ci, 'distortion_model', 'plumb_bob'),
        'distortion_coefficients': {'rows': 1, 'cols': len(ci.d), 'data': list(ci.d)},
        'rectification_matrix': {'rows': 3, 'cols': 3, 'data': list(ci.r) if ci.r else [1,0,0,0,1,0,0,0,1]},
        'projection_matrix': {'rows': 3, 'cols': 4, 'data': list(ci.p) if ci.p else [0.0]*12},
    }
    os.makedirs(os.path.dirname(yaml_path), exist_ok=True)
    with open(yaml_path, 'w') as f:
        yaml.safe_dump(data, f)
    return yaml_path

class SimpleCameraNode(Node):
    def __init__(self):
        super().__init__('simple_camera_node')

        # Parameters
        self.declare_parameter('device', 0)  # camera device index or path
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('camera_info_url', '')
        self.declare_parameter('frame_rate', 30.0)

        device = self.get_parameter('device').value
        self.frame_id = self.get_parameter('frame_id').value
        camera_info_url = self.get_parameter('camera_info_url').value
        fps = float(self.get_parameter('frame_rate').value)

        # YAML path
        if camera_info_url.startswith('file://'):
            self.yaml_path = camera_info_url[len('file://'):]
        elif camera_info_url:
            self.yaml_path = camera_info_url
        else:
            ns = self.get_namespace().strip('/')
            name = ns if ns else self.get_name()
            default_dir = str(Path.home() / '.ros' / 'camera_info')
            self.yaml_path = os.path.join(default_dir, f'{name}.yaml')

        # OpenCV capture
        self.bridge = CvBridge()
        self.cap = cv2.VideoCapture(int(device)) if str(device).isdigit() else cv2.VideoCapture(device)
        if not self.cap.isOpened():
            self.get_logger().error(f"Failed to open video device: {device}")
            raise RuntimeError("Camera open failed")

        # Publishers
        self.pub_image = self.create_publisher(Image, 'camera/image_raw', 10)
        self.pub_info = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Service (calibration tool writes here)
        self.srv = self.create_service(SetCameraInfo, 'set_camera_info', self.handle_set_camera_info)

        # Load existing camera_info if available
        self.camera_info_msg = None
        if os.path.exists(self.yaml_path):
            try:
                self.camera_info_msg = load_camera_info_from_yaml(self.yaml_path)
                if not self.camera_info_msg.header.frame_id:
                    self.camera_info_msg.header.frame_id = self.frame_id
                self.get_logger().info(f"Loaded camera_info from {self.yaml_path}")
            except Exception as e:
                self.get_logger().warning(f"Failed to load camera_info: {e}")

        # Use default CameraInfo if none
        if self.camera_info_msg is None:
            w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or 640
            h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or 480
            ci = CameraInfo()
            ci.width = w
            ci.height = h
            ci.k = [1.0, 0.0, w/2.0, 0.0, 1.0, h/2.0, 0.0, 0.0, 1.0]
            ci.d = []
            ci.r = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]

            ci.p = [1.0,0.0,w/2.0,0.0, 0.0,1.0,h/2.0,0.0, 0.0,0.0,1.0,0.0]
            ci.header.frame_id = self.frame_id
            self.camera_info_msg = ci
            self.get_logger().info("Using default (uninitalized) CameraInfo until calibrator updates it.")

        # Timer for publishing
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        self.get_logger().info(f"SimpleCameraNode started. Topics: /camera/image_raw, /camera/camera_info, Service: /camera/set_camera_info. YAML: {self.yaml_path}")

    def handle_set_camera_info(self, request, response):
        """Handle calibration tool's set_camera_info service call."""
        try:
            ci = request.camera_info
            if not ci.header.frame_id:
                ci.header.frame_id = self.frame_id
            if not ci.width or not ci.height:
                ci.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)) or ci.width
                ci.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)) or ci.height

            self.camera_info_msg = ci
            save_camera_info_to_yaml(ci, self.yaml_path, camera_name=self.get_namespace().strip('/') or self.get_name())

            response.success = True
            response.status_message = f"Camera info saved to {self.yaml_path}"
            self.get_logger().info(response.status_message)
        except Exception as e:
            response.success = False
            response.status_message = f"Error saving camera info: {e}"
            self.get_logger().error(response.status_message)
        return response

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn_once("Camera read failed (further warnings suppressed).")
            return

        ros_image = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        stamp = self.get_clock().now().to_msg()
        ros_image.header = Header()
        ros_image.header.stamp = stamp
        ros_image.header.frame_id = self.frame_id
        self.pub_image.publish(ros_image)

        ci = copy.deepcopy(self.camera_info_msg)
        ci.header.stamp = stamp
        ci.header.frame_id = self.frame_id
        self.pub_info.publish(ci)

def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SimpleCameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.cap.release()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
