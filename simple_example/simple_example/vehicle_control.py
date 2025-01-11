import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import numpy as np
import cv2
from cv_bridge import CvBridge

from geometry_msgs.msg import Twist
import time

from stable_baselines3 import PPO
import subprocess

class CameraDataAcquisition(Node):
    
    def __init__(self):
        super().__init__('CameraDataAcquisition')
        self.subscription = self.create_subscription(
            Image,
            '/world/sonoma/model/prius_hybrid_sensors/link/sensors/sensor/front_camera_sensor/image',
            self.listener_callback,
            20)
        self.subscription
        self.cvBridge = CvBridge()
        self.cv_image = np.zeros((3 ,224, 224))

    def listener_callback(self, msg):
        # self.get_logger().info('Image width: "%s"' % msg.width)
        # self.get_logger().info('Image height: "%s"' % msg.height)
        # self.get_logger().info('Image encoding: "%s"' % msg.encoding)
        self.cv_image = self.cvBridge.imgmsg_to_cv2(msg, 'bgr8').T
        # self.cv_image = self.cvBridge.imgmsg_to_cv2(msg, 'bgra8') # image for further processing

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publikuj co 1 sekundę
        # self.get_logger().info('Publishing /cmd_vel message')
        self.linear_x = 0.0
        self.angular_z = 0.0


    def publish_message(self):
        msg = Twist()
        msg.linear.x = float(self.linear_x)
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_z

        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published: {msg}')

class Control:
    def __init__(self, model_path):
        # Load PPO model
        self.model = PPO.load(model_path)
        
        self.cameraDataAcquisition = CameraDataAcquisition()
        self.minimal_publisher = MinimalPublisher()

        self.actions = {
        0: (1.0,0.0),
        1: (1.0,-0.5),
        2: (1.0,0.5),
        3: (2.0,0.0)
        }

    def control_loop(self):
        try:
            while rclpy.ok():

                # subprocess.run(["gz", "service", "-s", "/world/sonoma/control", "--reqtype", 
                #     "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", 
                #     "--timeout", "3000", "--req", "pause: false"])
                
                rclpy.spin_once(self.minimal_publisher)
                time.sleep(0.25)
                rclpy.spin_once(self.cameraDataAcquisition)
                
                # subprocess.run(["gz", "service", "-s", "/world/sonoma/control", "--reqtype", 
                #                 "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", 
                #                 "--timeout", "3000", "--req", "pause: true"])
                # Get camera image
                observation = self.cameraDataAcquisition.cv_image

                # Ensure observation is in the correct format
                print(observation.shape)
                if observation.shape == (3 ,224, 224):
                    observation = observation.astype(np.uint8)
                else:
                    print("Invalid observation shape, skipping action.")
                    continue

                # Get action from model
                action, _ = self.model.predict(observation, deterministic=True)

                self.minimal_publisher.linear_x, self.minimal_publisher.angular_z = self.actions[int(action)]

        except KeyboardInterrupt:
            print("Control loop interrupted.")
        

def main(args=None):
    rclpy.init(args=args)
    model_path = "model1"
    controller = Control(model_path)
    controller.control_loop()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
