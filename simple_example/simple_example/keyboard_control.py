import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios
import sys
import tty

class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.linear_speed = 1.0  # prędkość liniowa
        self.angular_speed = 0.5  # prędkość kątowa
        self.get_logger().info("Sterowanie robotem za pomocą klawiatury:")
        self.get_logger().info("Wciśnij 'w' (prosto), 's' (tył), 'a' (lewo), 'd' (prawo). Wciśnij 'q', aby zakończyć.")
        self.run()

    def get_key(self):
        """Odczytuje znak z klawiatury w trybie bezpośrednim."""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            key = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return key

    def run(self):
        """Główna pętla odczytu klawiatury i publikacji."""
        twist = Twist()

        while rclpy.ok():
            key = self.get_key()  # Odczyt znaku z klawiatury
            self.get_logger().info(f"Odczytano klawisz: {key}")

            # Ustawianie prędkości w zależności od klawisza
            if key == 'w':  # Prosto
                twist.linear.x = self.linear_speed
                twist.angular.z = 0.0
            elif key == 's':  # Tył
                twist.linear.x = -self.linear_speed
                twist.angular.z = 0.0
            elif key == 'a':  # Lewo
                twist.linear.x = 0.0
                twist.angular.z = self.angular_speed
            elif key == 'd':  # Prawo
                twist.linear.x = 0.0
                twist.angular.z = -self.angular_speed
            elif key == 'q':  # Wyjście
                self.get_logger().info("Zamykanie sterowania.")
                break
            else:
                # Nieznany klawisz – zatrzymanie robota
                self.get_logger().warning("Nieznany klawisz. Wciśnij w, s, a, d lub q.")
                twist.linear.x = 0.0
                twist.angular.z = 0.0

            # Publikacja wiadomości
            self.publisher_.publish(twist)
            self.get_logger().info(f"Publikacja: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Sterowanie przerwane przez użytkownika.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
