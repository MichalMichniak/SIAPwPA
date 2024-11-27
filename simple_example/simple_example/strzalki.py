import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info('Use arrow keys to move the robot. Press Ctrl+C to quit.')

    def get_key(self):
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            return sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def run(self):
        twist = Twist()
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == '\x1b':  # Start of an arrow key
                    next_key = self.get_key()
                    if next_key == '[':  # Arrow key prefix
                        direction = self.get_key()
                        if direction == 'A':  # Up arrow
                            twist.linear.x = 0.5
                            twist.angular.z = 0.0
                        elif direction == 'B':  # Down arrow
                            twist.linear.x = -0.5
                            twist.angular.z = 0.0
                        elif direction == 'C':  # Right arrow
                            twist.linear.x = 0.0
                            twist.angular.z = -0.5
                        elif direction == 'D':  # Left arrow
                            twist.linear.x = 0.0
                            twist.angular.z = 0.5
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                
                self.publisher.publish(twist)
        except KeyboardInterrupt:
            pass
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardTeleop()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
