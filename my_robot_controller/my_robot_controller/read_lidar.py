#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class read_lidar(Node):
    def __init__(self):
        super().__init__("read_lidar")
        self.lidar_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.lidar_callback,
            10
            )
        
        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.get_logger().info("start reading")

    def lidar_callback(self, rada : LaserScan):
        self.get_logger().info('angles: ' + str(rada.angle_min) + " : " + str(rada.angle_max) + " : " + str(rada.angle_increment))
        #self.get_logger().info('Intensities: ' + str(rada.ranges))

    def odom_callback(self, odm : Odometry):
        self.get_logger().info('odm: ' + str(odm.pose.pose.orientation.x))
        


def main(args=None):
    rclpy.init(args=args)
    node = read_lidar()
    rclpy.spin(node)
    rclpy.shutdown()