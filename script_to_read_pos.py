import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdometryListener(Node):
    def __init__(self):
        super().__init__('odometry_listener')
        self.subscription = self.create_subscription(
            Odometry,
            '/robot/odometry',  # Nazwa tematu
            self.odometry_callback,
            10  # QoS
        )

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")

def main(args=None):
    rclpy.init(args=args)
    node = OdometryListener()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
