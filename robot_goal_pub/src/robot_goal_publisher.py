#!/usr/bin/env python3
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.node import Node
# import tf_transformations
from transforms3d.euler import euler2quat

class GoalPublisher(Node):
    def __init__(self):
        super().__init__('goal_pose_publisher')
        self.publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
        timer_period = 5.0
        self.timer = self.create_timer(timer_period, self.publish_goal)

    def publish_goal(self):
        msg = PoseStamped()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.position.x = 2.0
        msg.pose.position.y = 3.0
        msg.pose.position.z = 0.0

        # Convert yaw to quaternion
        yaw = 1.57  # ~90 degrees
        quat = euler2quat(0, 0, yaw)
        msg.pose.orientation.x = quat[0]
        msg.pose.orientation.y = quat[1]
        msg.pose.orientation.z = quat[2]
        msg.pose.orientation.w = quat[3]

        self.publisher.publish(msg)
        self.get_logger().info('Published goal pose!')

if __name__ == '__main__':
    rclpy.init(args=None)
    node = GoalPublisher()
    rclpy.spin(node)
    rclpy.shutdown()