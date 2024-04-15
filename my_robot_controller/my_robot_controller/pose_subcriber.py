#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class pose_subcriber(Node):
    def __init__(self):
        super().__init__('pose_subcriberr')
        self.subscription = self.create_subscription(
            Pose,'/turtle1/pose',self.listener_callback,10)

    def listener_callback(self, msg : Pose):
        self.get_logger().info(str(msg))


def main(args=None):
    rclpy.init(args=args)
    node = pose_subcriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()