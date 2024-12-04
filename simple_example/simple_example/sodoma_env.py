import gym
from gym import spaces
import os

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image

import numpy as np
import cv2 as cv
from cv_bridge import CvBridge
from stable_baselines3 import PPO
from stable_baselines3.ppo import CnnPolicy, MlpPolicy
import subprocess
import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
import time

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
        self.cv_image = np.zeros((800, 800, 4))

    def listener_callback(self, msg):
        self.get_logger().info('Image width: "%s"' % msg.width)
        self.get_logger().info('Image height: "%s"' % msg.height)
        self.get_logger().info('Image encoding: "%s"' % msg.encoding)
        self.cv_image = self.cvBridge.imgmsg_to_cv2(msg, 'bgra8') # image for further processing



class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publikuj co 1 sekundę
        self.get_logger().info('Publishing /cmd_vel message')
        self.linear_x = 0
        self.angular_z = 0


    def publish_message(self):
        msg = Twist()
        msg.linear.x = self.linear_x
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = self.angular_z

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published: {msg}')


class SodomaAndGomora(gym.Env):
    """Custom Environment that follows gym interface"""

    def __init__(self):
      super(SodomaAndGomora, self).__init__()
      self.cameraDataAcquisition = CameraDataAcquisition()
      self.minimal_publisher = MinimalPublisher()
      # Define action and observation space
      # They must be gym.spaces objects
      # Example when using discrete actions:
      self.action_space = spaces.Discrete(4)
      # parametr do done
      self.max_steps = 128
      self.curr_steps = 0
      # Example for using image as input (channel-first; channel-last also works):
      self.observation_space = spaces.Box(low=0, high=255,
                        shape=(800, 800, 4), dtype=np.uint8)
      self.actions = {
        0: (1.0,0.0),
        1: (1.0,-0.5),
        2: (1.0,0.5),
        3: (-0.5,0.0)
        }

    def step(self, action):
        #TODO: sterowanie
        self.minimal_publisher.linear_x, self.minimal_publisher.angular_z = self.actions[action]
        
        #TODO: tutaj plougin na step symulation
        subprocess.run(["gz", "service", "-s", "/world/sonoma/control", "--reqtype", 
                    "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", 
                    "--timeout", "3000", "--req", "pause: false"])
        rclpy.spin_once(self.minimal_publisher)
        time.sleep(1)
        rclpy.spin_once(self.cameraDataAcquisition)
        subprocess.run(["gz", "service", "-s", "/world/sonoma/control", "--reqtype", 
                        "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", 
                        "--timeout", "3000", "--req", "pause: true"])
        #   os.system("gz service -s /world/sonoma/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'")
        #   rclpy.spin_once(self.cameraDataAcquisition)
        #   os.system("gz service -s /world/sonoma/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'")
        observation = self.cameraDataAcquisition.cv_image
        reward = 0.01 #TODO
        done = False #TODO parametr oznaczający kiedy restart
        self.curr_steps+=1
        if(self.curr_steps > self.max_steps):
            done = True
            self.curr_steps=0
        
        info = {}
        return observation, reward, done, info
    
    def reset(self):
      # TODO: to jest upośledzone
      os.system("gz service -s /world/sonoma/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}' && gz service -s /world/sonoma/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'")
      self.curr_steps = 0
      os.system("gz service -s /world/sonoma/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'")
      rclpy.spin_once(self.cameraDataAcquisition, timeout_sec=0.1)
      os.system("gz service -s /world/sonoma/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'")
      observation = self.cameraDataAcquisition.cv_image
      return observation # reward, done, info can't be included



def main(args=None):
    rclpy.init(args=args)
    env = SodomaAndGomora()
    model = PPO(CnnPolicy, env, verbose=3, batch_size=64)
    model.learn(total_timesteps=2, reset_num_timesteps=False)
    model.save(f"model{1}")
    env.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()