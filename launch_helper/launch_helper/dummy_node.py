#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import ros2_lib
import ros2_lib.launch_helper

def main(args=None):
    rclpy.init(args=args)

    dummy_node = Node('dummy_node')
    ld = ros2_lib.launch_helper.LaunchHelper()

    rclpy.spin(dummy_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dummy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
