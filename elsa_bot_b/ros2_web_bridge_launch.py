
import os
import sys
import subprocess

import rclpy
from rclpy.node import Node

def main(args=None):
    rclpy.init(args=args)

    print("Running ros2 web bridge")
    p = subprocess.run(['node', '/home/ubuntu/ros2-web-bridge/bin/rosbridge.js'])

    rclpy.spin(depthai_publisher)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
