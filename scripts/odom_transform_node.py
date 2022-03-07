#! /usr/bin/env python3

import numpy as np
from dh_transform import transform

# ros
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener

# msgs
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped




class OdomTransform(Node):
    def __init__(self):
        super().__init__('odom_transform')
        self.counter = 0
        # get parameters
        self.declare_parameter('odom_topic',   '/t265_camera/odom')
        self.declare_parameter('pose_topic',   '/t265_camera/odom/to_base')
        self.declare_parameter('target_frame', 'base_link')

        self.odom_topic = self.get_parameter('odom_topic').value
        self.pose_topic = self.get_parameter('pose_topic').value
        self.target_frame = self.get_parameter('target_frame').value

        self.get_logger().info(f'Projecting {self.odom_topic} to frame {self.target_frame} on topic {self.pose_topic}')

        # initialize tf2
        self.buffer = Buffer()
        self.tf_listener = TransformListener(self.buffer, self)

        # initialize ros communication
        self.odom_sub = self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 1)
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, self.pose_topic, 1)


    def odom_callback(self, odom: Odometry) -> type(None):
        if self.counter < 10:
            self.counter += 1
            return
        else:
            self.counter = 0
        # create PoseWithCovarianceStamped from Odometry
        pose                 = PoseWithCovarianceStamped()
        pose.header          = odom.header
        pose.pose            = odom.pose
        pose.pose.covariance = odom.pose.covariance

        # get transform from target_frame to original one
        tf = TransformStamped()
        try:
            tf  = self.buffer.lookup_transform(odom.child_frame_id, self.target_frame, rclpy.time.Time())
        except TransformException:
            self.get_logger().warn(f'Could not get transform from {odom.child_frame_id} to {self.target_frame}')
            return

        pose.pose.pose = transform(pose.pose.pose, tf.transform)
        self.pose_pub.publish(pose)

def main(args=None) -> type(None):
    rclpy.init(args=args)
    odomTransform = OdomTransform()
    rclpy.spin(odomTransform)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
