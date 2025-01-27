import rclpy
import csv
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometryListener(Node):
    def __init__(self, lock = None, state = None):
        super().__init__('odometry_listener')
        
        self.coords = [0,0]
        self.subscription = self.create_subscription(
        Odometry,
        '/robot/odometry', # Nazwa tematu
        self.odometry_callback,
        20 # QoS
        )
        self.lock = lock
        self.state = state

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")
        if self.lock is not None:
            self.state.wait()
            with self.lock:
                self.coords = [position.x, position.y]
        else:
            self.coords = [position.x, position.y]

def main(args=None):
    rclpy.init(args=args)
    node = OdometryListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()