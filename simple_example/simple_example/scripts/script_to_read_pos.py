# import rclpy
# from rclpy.node import Node
# from nav_msgs.msg import Odometry

# class OdometryListener(Node):
#     def __init__(self):
#         super().__init__('odometry_listener')
#         self.subscription = self.create_subscription(
#             Odometry,
#             '/robot/odometry',  # Nazwa tematu
#             self.odometry_callback,
#             10  # QoS
#         )

#     def odometry_callback(self, msg):
#         position = msg.pose.pose.position
#         self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdometryListener()
#     rclpy.spin(node)
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()

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
        # with open('output4.csv', 'a', newline='') as f_output:
        #     csv_output = csv.writer(f_output)
        #     # csv_output.writerow(['lat', 'long'])
        #     csv_output.writerows(self.coords)


# def write_postion(self):
# with open('output.csv', 'w', newline='') as f_output:
# csv_output = csv.writer(f_output)
# # csv_output.writerow(['lat', 'long'])
# csv_output.writerows(self.coords)

def main(args=None):
    rclpy.init(args=args)
    node = OdometryListener()
    rclpy.spin(node)
    rclpy.shutdown()
# node.write_postion()

if __name__ == '__main__':
    main()