#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
 
class HokuyoToSick(Node):
    def __init__(self):
        super().__init__('hokuyo_to_sick')
        # Subscribe to both sensor topics.
        # self.subscription_sick = self.create_subscription(
        #     LaserScan,
        #     '/scan_hokuyo',          # topic for Sick LMS100
        #     self.sick_callback,
        #     10)
        self.subscription_hokuyo = self.create_subscription(
            LaserScan,
            '/scan',        # topic for Hokuyo URG
            self.sick_callback,
            10)
        # Publisher for the new (substituted) Sick scan.
        self.publisher_ = self.create_publisher(
            LaserScan,
            '/scan3',
            10)
 
        # Store the latest Sick message to get its parameters.
        self.latest_sick = None
 
    def sick_callback(self, msg: LaserScan):
        # Save the latest Sick message to use its parameters.
        print("entrou")
        msg.header.frame_id = "hokuyo_ground_link"
        self.publisher_.publish(msg)
 
def main(args=None):
    rclpy.init(args=args)
    node = HokuyoToSick()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
 
if __name__ == '__main__':
    main()