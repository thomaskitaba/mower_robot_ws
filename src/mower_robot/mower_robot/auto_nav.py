#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time

class AutoNavNode(Node):
    def __init__(self):
        super().__init__('auto_nav')
        self.action_client = ActionClient(self, NavigateToPose, '/mower_robot/navigate_to_pose')
        self.cmd_vel_pub = self.create_publisher(Twist, '/mower_robot/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/mower_robot/scan', self.scan_callback, 10)
        self.min_distance = 0.5  # Stop if obstacle closer than 0.5m
        self.obstacle_detected = False
        self.get_logger().info('AutoNav node started, waiting 5 seconds before sending goal...')
        self.timer = self.create_timer(5.0, self.send_goal)  # Send goal after 5 seconds

    def scan_callback(self, msg):
        # Check for obstacles within min_distance
        ranges = msg.ranges
        valid_ranges = [r for r in ranges if r > 0.0 and r < msg.range_max]  # Filter invalid ranges
        if valid_ranges and min(valid_ranges) < self.min_distance:
            if not self.obstacle_detected:
                self.get_logger().info('Obstacle detected! Stopping robot.')
                self.obstacle_detected = True
                # Publish zero velocity to stop
                stop_msg = Twist()
                stop_msg.linear.x = 0.0
                stop_msg.angular.z = 0.0
                self.cmd_vel_pub.publish(stop_msg)
        else:
            self.obstacle_detected = False

    def send_goal(self):
        self.timer.cancel()  # Run once
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Navigation server not available!')
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 4.0
        goal_msg.pose.pose.position.y = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info('Sending navigation goal to (4, 0, 0)')
        self.action_client.send_goal_async(goal_msg)
        self.get_logger().info('Goal sent!')

def main(args=None):
    rclpy.init(args=args)
    node = AutoNavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
