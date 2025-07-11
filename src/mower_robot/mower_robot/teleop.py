#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import sys
import select
import termios
import tty

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop')
        self.publisher = self.create_publisher(Twist, '/mower_robot/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/mower_robot/scan', self.scan_callback, 10)
        self.settings = termios.tcgetattr(sys.stdin)
        self.min_distance = 1.0  # Stop if obstacle closer than 1.0m
        self.border_threshold = 2.0  # Range jump for border detection
        self.max_range = 10.0  # LIDAR max range
        self.obstacle_detected = False
        self.border_detected = False
        self.linear_speed = 0.5  # m/s
        self.angular_speed = 2.0  # rad/s (increased for faster turning)
        self.key_timeout = 0.1  # seconds
        self.stop_timer = self.create_timer(0.01, self.publish_stop)  # 100Hz stop commands
        self.get_logger().info('Teleop node started. Use arrow keys to move (Up/Down: forward/back, Left/Right: turn). Press Ctrl+C to quit.')

    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], self.key_timeout)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def scan_callback(self, msg):
        # Filter valid ranges (all directions)
        ranges = msg.ranges
        valid_ranges = [
            r for r in ranges
            if not (r == float('inf') or r != r or r < msg.range_min or r > msg.range_max)
        ]
        inf_ranges = [r for r in ranges if r == float('inf')]

        # Detect obstacle or border
        if valid_ranges:
            min_range = min(valid_ranges)
            max_range = max(valid_ranges)
            self.get_logger().info(f'Min range: {min_range:.2f}m, Max range: {max_range:.2f}m, Inf ranges: {len(inf_ranges)}')

            # Obstacle detection
            if min_range < self.min_distance:
                if not self.obstacle_detected and not self.border_detected:
                    self.get_logger().info(f'Obstacle detected at {min_range:.2f}m! Stopping robot.')
                    self.obstacle_detected = True
            # Border detection (range jump or near max range)
            elif max_range - min_range > self.border_threshold or max_range > 0.9 * self.max_range or len(inf_ranges) > 10:
                if not self.border_detected and not self.obstacle_detected:
                    self.get_logger().info(f'Border detected! Range jump: {max_range - min_range:.2f}m or max range: {max_range:.2f}m or inf count: {len(inf_ranges)}. Stopping robot.')
                    self.border_detected = True
            else:
                if self.obstacle_detected or self.border_detected:
                    self.get_logger().info('Obstacle or border cleared. Resuming manual control.')
                    self.obstacle_detected = False
                    self.border_detected = False
        else:
            self.get_logger().info('No valid LIDAR ranges detected')

    def publish_stop(self):
        # Publish stop commands during obstacle/border detection
        if self.obstacle_detected or self.border_detected:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.publisher.publish(stop_msg)

    def run(self):
        twist = Twist()
        try:
            while True:
                key = self.get_key()
                if not (self.obstacle_detected or self.border_detected):
                    if key == '\x1b':  # Escape sequence for arrow keys
                        key = sys.stdin.read(2)
                        if key == '[A':  # Up arrow
                            twist.linear.x = self.linear_speed
                            twist.angular.z = 0.0
                        elif key == '[B':  # Down arrow
                            twist.linear.x = -self.linear_speed
                            twist.angular.z = 0.0
                        elif key == '[D':  # Left arrow
                            twist.linear.x = 0.0
                            twist.angular.z = self.angular_speed
                        elif key == '[C':  # Right arrow
                            twist.linear.x = 0.0
                            twist.angular.z = -self.angular_speed
                        else:
                            twist.linear.x = 0.0
                            twist.angular.z = 0.0
                    else:
                        twist.linear.x = 0.0
                        twist.angular.z = 0.0
                        if key == '\x03':  # Ctrl+C
                            break
                    self.publisher.publish(twist)
                else:
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.publisher.publish(twist)
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)

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