#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_publisher')
        self.publisher = self.create_publisher(PoseWithCovarianceStamped, '/mower_robot/initialpose', 10)
        self.timer = self.create_timer(1.0, self.publish_pose)
        self.get_logger().info('Publishing initial pose to /mower_robot/initialpose')

    def publish_pose(self):
        pose = PoseWithCovarianceStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.pose.position.x = 0.0
        pose.pose.pose.position.y = 0.0
        pose.pose.pose.position.z = 0.0
        pose.pose.pose.orientation.w = 1.0
        self.publisher.publish(pose)
        self.get_logger().info('Initial pose published')
        self.timer.cancel()  # Publish once and stop

def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()