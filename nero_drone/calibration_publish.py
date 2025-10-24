#!/usr/bin/env python3
# CalibrationPublisher: ROS2 node that loads camera calibration parameters from a .pkl file 
# and publishes them periodically as a CameraInfo message for use by other nodes.
# Author: Brayan Saldarriaga-Mesa (bsaldarriaga@inaut.unsj.edu.ar), in collaboration with UFV.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import pickle
import os
from ament_index_python.packages import get_package_share_directory

class CalibrationPublisher(Node):
    def __init__(self):
        super().__init__('calibration_publisher')

        self.pub = self.create_publisher(CameraInfo, '/bebop/camera/calibration_info', 1)

        try:
            pkg_share = get_package_share_directory('nero_drone')
            pkl_path = os.path.join(pkg_share, 'config', 'calibration_data.pkl')

            if not os.path.exists(pkl_path):
                self.get_logger().error(f"Calibration file not found: {pkl_path}")
                return

            with open(pkl_path, 'rb') as f:
                data = pickle.load(f)

            cam = data.get('camera_matrix')
            dist = data.get('distortion_coefficients')

            if cam is None or dist is None:
                self.get_logger().error("Calibration file missing 'camera_matrix' or 'distortion_coefficients'")
                return

            msg = CameraInfo()
            msg.header.frame_id = "camera_optical"
            msg.distortion_model = "plumb_bob"
            msg.d = dist.flatten().tolist()
            msg.k = cam.flatten().tolist()
            msg.height = 480
            msg.width = 856

            self.timer = self.create_timer(1.0, lambda: self.pub.publish(msg))
            self.get_logger().info(f"Publishing calibration data from {pkl_path}")

        except Exception as e:
            self.get_logger().error(f"Error loading calibration_data.pkl: {e}")

def main():
    rclpy.init()
    node = CalibrationPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
