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

# import rclpy
# import csv
# from rclpy.node import Node
# from nav_msgs.msg import Odometry

# class OdometryListener(Node):
#     def __init__(self):
#         super().__init__('odometry_listener')
#         self.coords = []
#         self.subscription = self.create_subscription(
#         Odometry,
#         '/robot/odometry', # Nazwa tematu
#         self.odometry_callback,
#         10 # QoS
#         )
#     def odometry_callback(self, msg):
#         position = msg.pose.pose.position
#         self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")
#         self.coords.append((position.x, position.y))
#         with open('output4.csv', 'a', newline='') as f_output:
#             csv_output = csv.writer(f_output)
#             # csv_output.writerow(['lat', 'long'])
#             csv_output.writerows(self.coords)


# # def write_postion(self):
# # with open('output.csv', 'w', newline='') as f_output:
# # csv_output = csv.writer(f_output)
# # # csv_output.writerow(['lat', 'long'])
# # csv_output.writerows(self.coords)

# def main(args=None):
#     rclpy.init(args=args)
#     node = OdometryListener()
#     rclpy.spin(node)
#     rclpy.shutdown()
# # node.write_postion()

# if __name__ == '__main__':
#     main()

import rclpy
import csv
from rclpy.node import Node
from nav_msgs.msg import Odometry
import numpy as np

class OdometryListener(Node):
    def __init__(self):
        super().__init__('odometry_listener')
        self.coords = []
        self.last_position = None  # Przechowuje ostatnią zapisaną pozycję
        self.subscription = self.create_subscription(
            Odometry,
            '/robot/odometry',
            self.odometry_callback,
            10
        )

    def odometry_callback(self, msg):
        position = msg.pose.pose.position
        current_position = (position.x, position.y)
        
        # Sprawdzenie zmiany pozycji
        if self.last_position is None or self.has_significant_change(self.last_position, current_position):
        # if self.last_position is None or 1:
            self.get_logger().info(f"Position: x={position.x}, y={position.y}, z={position.z}")
            self.coords.append(current_position)
            self.last_position = current_position  # Zaktualizowanie ostatniej pozycji
            
            # Zapis do pliku
            with open('output4.csv', 'a', newline='') as f_output:
                csv_output = csv.writer(f_output)
                csv_output.writerow(current_position)

    def has_significant_change(self, last_pos, current_pos):
        # Ustal próg minimalnej zmiany
        threshold = 0.8  # Możesz dostosować wartość (np. 0.1 metra)
        dx = (current_pos[0] - last_pos[0])**2
        dy = (current_pos[1] - last_pos[1])**2

        return np.sqrt(dx+ dy) > threshold
    


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
