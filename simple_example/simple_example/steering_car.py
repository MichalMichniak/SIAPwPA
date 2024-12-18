from stable_baselines3 import PPO
import rclpy
from rclpy.node import Node
import cv2

import sodoma_env

def main(args=None):
  rclpy.init(args=args)
  env = sodoma_env.SodomaAndGomora()
  model = PPO.load("model1") #TODO: model1
  obs = env.reset()
  while True:
    action, _states = model.predict(obs, deterministic=True)
    obs, rewards, done, info = env.step(action)
    if done:
      obs = env.reset()

    if cv2.waitKey(1) & 0xFF == ord('q'):
      break

  env.close()
  rclpy.shutdown()

if __name__ == "__main__":
  main()