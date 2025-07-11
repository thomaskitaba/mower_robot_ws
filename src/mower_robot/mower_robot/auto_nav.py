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
        self.min_distance = 1.0  # Stop if obstacle closer than 1.0m
        self.border_threshold = 2.0  # Range jump for border detection
        self.obstacle_detected = False
        self.border_detected = False
        self.goal_handle = None
        self.stop_timer = self.create_timer(0.05, self.publish_stop)  # 20Hz stop commands
        self.get_logger().info('AutoNav node started, waiting 5 seconds before sending goal...')
        self.timer = self.create_timer(5.0, self.send_goal)  # Send goal after 5 seconds

    def scan_callback(self, msg):
        # Filter valid ranges (all directions)
        ranges = msg.ranges
        valid_ranges = [
            r for r in ranges
            if not (r == float('inf') or r != r or r < msg.range_min or r > msg.range_max)
        ]

        # Detect obstacle or border
        if valid_ranges:
            min_range = min(valid_ranges)
            self.get_logger().info(f'Min range: {min_range:.2f}m')
            
            # Obstacle detection
            if min_range < self.min_distance:
                if not self.obstacle_detected and not self.border_detected:
                    self.get_logger().info(f'Obstacle detected at {min_range:.2f}m! Stopping robot.')
                    self.obstacle_detected = True
                    if self.goal_handle:
                        self.get_logger().info('Canceling navigation goal')
                        self.action_client.cancel_goal_async()
            
            # Border detection (significant range jump)
            max_range = max(valid_ranges)
            if max_range - min_range > self.border_threshold:
                if not self.border_detected and not self.obstacle_detected:
                    self.get_logger().info(f'Border detected! Range jump: {max_range - min_range:.2f}m. Stopping robot.')
                    self.border_detected = True
                    if self.goal_handle:
                        self.get_logger().info('Canceling navigation goal')
                        self.action_client.cancel_goal_async()
            else:
                if self.obstacle_detected or self.border_detected:
                    self.get_logger().info('Obstacle or border cleared. Resending navigation goal.')
                    self.obstacle_detected = False
                    self.border_detected = False
                    self.send_goal()
        else:
            self.get_logger().info('No valid LIDAR ranges detected')

    def publish_stop(self):
        # Publish stop commands during obstacle/border detection
        if self.obstacle_detected or self.border_detected:
            stop_msg = Twist()
            stop_msg.linear.x = 0.0
            stop_msg.angular.z = 0.0
            self.cmd_vel_pub.publish(stop_msg)

    def send_goal(self):
        self.timer.cancel()  # Run once
        max_retries = 5
        retry_delay = 2.0
        for attempt in range(max_retries):
            if self.action_client.wait_for_server(timeout_sec=10.0):
                goal_msg = NavigateToPose.Goal()
                goal_msg.pose.header.frame_id = 'map'
                goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
                goal_msg.pose.pose.position.x = 4.0
                goal_msg.pose.pose.position.y = 0.0
                goal_msg.pose.pose.orientation.w = 1.0

                self.get_logger().info('Sending navigation goal to (4, 0, 0)')
                future = self.action_client.send_goal_async(goal_msg)
                future.add_done_callback(self.goal_response_callback)
                return
            else:
                self.get_logger().warn(f'Navigation server not available, retry {attempt + 1}/{max_retries}')
                time.sleep(retry_delay)
        self.get_logger().error('Failed to connect to navigation server after retries')

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected!')
            return
        self.get_logger().info('Navigation goal accepted')

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