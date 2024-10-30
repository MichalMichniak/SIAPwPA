import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge

class CameraDataAcquisition(Node):
    
    def __init__(self):
        super().__init__('CameraDataAcquisition')
        self.subscription = self.create_subscription(
            Image,
            '/world/sonoma/model/prius_hybrid_sensors/link/sensors/sensor/front_camera_sensor/image',
            self.listener_callback,
            10)
        self.subscription
        self.cvBridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Image width: "%s"' % msg.width)
        self.get_logger().info('Image height: "%s"' % msg.height)
        self.get_logger().info('Image encoding: "%s"' % msg.encoding)
        self.cv_image = self.cvBridge.imgmsg_to_cv2(msg, 'bgra8') # image for further processing

def main(args=None):
    rclpy.init(args=args)

    cameraDataAcquisition = CameraDataAcquisition()
    rclpy.spin(cameraDataAcquisition)

    cameraDataAcquisition.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()