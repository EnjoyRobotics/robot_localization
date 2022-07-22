#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point
from nav_msgs.msg import Odometry
from scipy.spatial.transform import Rotation


class QuaternionToEuler(Node):
    def __init__(self):
        super().__init__('quaternion_to_euler')

        topic = self.declare_parameter('topic', '').value
        self.pub = self.create_publisher(Point, f'/euler_angles/{topic}', 10)
        self.sub = self.create_subscription(Odometry, f'/{topic}', self.callback, 10)

    def callback(self, msg):
        R = Rotation([
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ])

        euler = R.as_euler('XYZ', degrees=False)

        point = Point()
        point.x = euler[0]
        point.y = euler[1]
        point.z = euler[2]

        self.pub.publish(point)


def main(args=None):
    rclpy.init(args=args)

    rclpy.spin(QuaternionToEuler())


if __name__ == '__main__':
    main()
