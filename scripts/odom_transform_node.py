#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer

# msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped


class OdomTransform(Node):
    def __init(self):
        super().__init__('odom_transform')

        # get parameters
        self.declare_parameter('odom_topic')
        self.declare_parameter('pose_topic')
        self.declare_parameter('target_frame')

        self.odom_topic = self.get_parameter('odom_topic')
        self.pose_topic = self.get_parameter('pose_topic')
        self.target_frame = self.get_parameter('target_frame')

        self.get_logger().info(f'Projecting {odom_topic} to frame {target_frame} on topic {pose_topic}')

        # initialize tf2
        self.buffer = Buffer()

        # initialize ros communication
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, pose_topic)

    def odom_callback(self, odom: Odometry) -> type(None):
        pose_before = PoseStamped()
        pose_before.header = odom.header
        pose_before.pose.pose.position.x    = odom.pose.pose.position.x
        pose_before.pose.pose.position.y    = odom.pose.pose.position.y
        pose_before.pose.pose.position.z    = odom.pose.pose.position.z
        pose_before.pose.pose.orientation.x = odom.pose.pose.orientation.x
        pose_before.pose.pose.orientation.y = odom.pose.pose.orientation.y
        pose_before.pose.pose.orientation.z = odom.pose.pose.orientation.z
        pose_before.pose.pose.orientation.w = odom.pose.pose.orientation.w

        pose = self.buffer.transform(pose_before, self.target_frame)

        posecs = PoseWithCovarianceStamped()
        posecs.header = pose.header
        posecs.pose.pose = pose.pose
        posecs.pose.covariance = odom.pose.covariance

        self.pose_pub.publish(posecs)

def main(args=None) -> type(None):
    rclpy.init(args=args)
    odomTransform = OdomTransform()
    rclpy.spin(odomTransform)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
