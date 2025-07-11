#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, '/mower_robot/cmd_vel', 10)
        self.get_logger().info("Use Arrow keys to move, Spacebar to stop, Ctrl+C to quit")
        self.settings = termios.tcgetattr(sys.stdin)
        self.moveBindings = {
            '\x1b[A': (1.0, 0.0),   # Up arrow: Forward
            '\x1b[B': (-1.0, 0.0),  # Down arrow: Backward
            '\x1b[D': (0.0, 1.0),   # Left arrow: Turn left
            '\x1b[C': (0.0, -1.0),  # Right arrow: Turn right
            ' ': (0.0, 0.0),        # Spacebar: Stop
        }
        self.speed = 0.5
        self.turn = 1.0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        # Handle arrow keys (escape sequences)
        if key == '\x1b':  # Escape character
            key += sys.stdin.read(2)  # Read [A, [B, [C, or [D
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        twist = Twist()
        while True:
            key = self.getKey()
            if key in self.moveBindings:
                twist.linear.x = self.moveBindings[key][0] * self.speed
                twist.angular.z = self.moveBindings[key][1] * self.turn
            else:
                twist = Twist()
                if key == '\x03':  # Ctrl+C
                    break
            self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = TeleopNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()