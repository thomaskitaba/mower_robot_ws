#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
import time

class AutoNavNode(Node):
    def __init__(self):
        super().__init__('auto_nav_node')
        self.action_client = ActionClient(self, NavigateToPose, '/mower_robot/navigate_to_pose')
        self.get_logger().info('Waiting for Nav2 action server...')
        self.action_client.wait_for_server()
        self.get_logger().info('Nav2 action server ready. Waiting 5 seconds before sending goal...')
        self.timer = self.create_timer(5.0, self.send_goal)  # Trigger after 5 seconds

    def send_goal(self):
        self.timer.cancel()  # Stop the timer after first execution
        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = 4.0  # Navigate to (4, 0, 0) within grass area
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0
        goal_msg.pose = goal_pose
        
        self.get_logger().info('Sending navigation goal to (4, 0, 0)')
        self.action_client.send_goal_async(goal_msg).add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        self.get_logger().info('Goal accepted!')
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation complete!')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = AutoNavNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
