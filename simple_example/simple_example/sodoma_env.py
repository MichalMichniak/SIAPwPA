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
import pandas as pd
from scripts.reward_function import GetDistances
from scripts.script_to_read_pos import OdometryListener
import threading



class CameraDataAcquisition(Node):
    
    def __init__(self, lock, state):
        super().__init__('CameraDataAcquisition')
        
        self.subscription = self.create_subscription(
            Image,
            '/world/sonoma/model/prius_hybrid_sensors/link/sensors/sensor/front_camera_sensor/image',
            self.listener_callback, 20)
        self.subscription
        self.cvBridge = CvBridge()
        self.cv_image = np.zeros((3, 224, 224))
        self.lock = lock
        self.state = state

    def listener_callback(self, msg):
        self.get_logger().info('Image width: "%s"' % msg.width)
        if self.lock:
            self.state.wait()
            with self.lock:
                self.cv_image = self.cvBridge.imgmsg_to_cv2(msg, 'bgr8').T # image for further processing

class MinimalPublisher(Node):
    def __init__(self, lock, state):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.publish_message)  # Publikuj co 1 sekundę
        self.linear_x = 0.0
        self.angular_z = 0.0
        self.lock = lock
        self.state = state

    def publish_message(self):
        if self.lock is not None:
            self.state.wait()
            with self.lock:
                msg = Twist()
                msg.linear.x = self.linear_x
                msg.linear.y = 0.0
                msg.linear.z = 0.0
                msg.angular.x = 0.0
                msg.angular.y = 0.0
                msg.angular.z = self.angular_z
                self.publisher_.publish(msg)
        else:
            print("Not async")
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
      self.wew = pd.read_csv("/home/developer/ros2_ws/src/simple_example/simple_example/scripts/wewnetrzne_git.csv")
      self.zew = pd.read_csv("/home/developer/ros2_ws/src/simple_example/simple_example/scripts/zewnetrzne_git.csv")
      
      # inicialization of read mutexes
      self.camera_lck = threading.Lock()
      self.position_lck = threading.Lock()
      self.action_lck = threading.Lock()
      self.state = threading.Event() ## flag set running
      self.state.clear()
      
      self.find_len = GetDistances(self.wew, self.zew, 100)
      self.node = OdometryListener(self.position_lck,self.state)
      self.cameraDataAcquisition = CameraDataAcquisition(self.camera_lck,self.state)
      self.minimal_publisher = MinimalPublisher(self.action_lck,self.state)
      self.executor = rclpy.executors.MultiThreadedExecutor()
      self.executor.add_node(self.minimal_publisher)
      self.executor.add_node(self.cameraDataAcquisition)
      self.executor.add_node(self.node)
      self.exe_thread = threading.Thread(target = self.executor.spin, daemon=True)
      self.prev_coords = (0,0)
      self.exe_thread.start()
      self.action_space = spaces.Discrete(4)
      self.observation_space = spaces.Box(low=0, high=255,
                        shape=(3 ,224, 224), dtype=np.uint8)
      self.actions = {
        0: (1.0,0.0),
        1: (1.0,-0.5),
        2: (1.0,0.5),
        3: (2.0,0.0)
        }

    def step(self, action):
        self.minimal_publisher.linear_x, self.minimal_publisher.angular_z = self.actions[action]
        # symulation:
        self.state.set()
        subprocess.run(["gz", "service", "-s", "/world/sonoma/control", "--reqtype", 
                    "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", 
                    "--timeout", "3000", "--req", "pause: false"])
        time.sleep(0.5)
        self.state.clear()
        subprocess.run(["gz", "service", "-s", "/world/sonoma/control", "--reqtype", 
                        "gz.msgs.WorldControl", "--reptype", "gz.msgs.Boolean", 
                        "--timeout", "3000", "--req", "pause: true"])
        # get obserwations:
        with self.camera_lck:
            observation = self.cameraDataAcquisition.cv_image
        with self.position_lck:
            coords = self.node.coords

        # compute rewards:
        left, right, dist= self.find_len.get_rewards(coords[0],coords[1])
        minimize = ((left+right)/2) - min(left,right)
        reward = np.exp(-minimize**2/7)-0.5

        # reset criterion:
        done = left+0.7>dist or right+0.7>dist
        if done:
            reward -= 1
        self.prev_coords = self.node.coords

        info = {}
        return observation, reward, done, info
    
    def reset(self):
      # reset function
      os.system("gz service -s /world/sonoma/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'reset: {all: true}' && gz service -s /world/sonoma/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'")
      os.system("gz service -s /world/sonoma/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: false'")
      os.system("gz service -s /world/sonoma/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean --timeout 3000 --req 'pause: true'")
      with self.camera_lck:
        observation = self.cameraDataAcquisition.cv_image
      return observation



def main(args=None):
    rclpy.init(args=args)
    env = SodomaAndGomora()
    # chose to make new model or train existing one
    model = PPO(CnnPolicy, env, verbose=3, batch_size=32)
    # model = PPO.load(f"model{1}",env=env)
    for i in range(1000):
        model.learn(total_timesteps=100, reset_num_timesteps=False)
        model.save(f"model{1}")
    env.close()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
