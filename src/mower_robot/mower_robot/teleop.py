#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_node')
        self.publisher = self.create_publisher(Twist, '/mower_robot/cmd_vel', 10)
        self.get_logger().info("Use WASD to move, X to stop, Ctrl+C to quit")
        self.settings = termios.tcgetattr(sys.stdin)
        self.moveBindings = {
            'w': (1.0, 0.0),  # Forward
            's': (-1.0, 0.0), # Backward
            'a': (0.0, 1.0),  # Left
            'd': (0.0, -1.0), # Right
            'x': (0.0, 0.0),  # Stop
        }
        self.speed = 0.5
        self.turn = 1.0

    def getKey(self):
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
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
