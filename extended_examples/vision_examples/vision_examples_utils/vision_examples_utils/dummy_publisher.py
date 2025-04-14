#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sensor_msgs.msg

import cv2
import numpy as np
from cv_bridge import CvBridge


class DummyPublisher(Node):

    def __init__(self):
        super().__init__('dummy_publisher')
        self.get_logger().info('Initializing')
        self.publisher = self.create_publisher(sensor_msgs.msg.Image, 'dummy_image', 10)
        self.timer = self.create_timer(0.1, self.callback_timer)
        self.counter = 0
        self.get_logger().info('Initialized')

    def callback_timer(self):
        cv_bridge = CvBridge()
        width = 1280
        height = 720
        image = np.zeros((height, width, 3), np.uint8)
        cv2.putText(image, f'{self.get_clock().now().seconds_nanoseconds()}', (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (51, 153, 255), 2)
        self.counter += 1

        msg_image = cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg_image.header.stamp.sec, msg_image.header.stamp.nanosec = self.get_clock().now().seconds_nanoseconds()
        msg_image.height = height
        msg_image.width = width
        msg_image.encoding = 'bgr8'

        self.publisher.publish(msg_image)
        self.get_logger().info('Published new image.')


def main(args=None):
    rclpy.init(args=args)

    dummy_publisher = DummyPublisher()

    rclpy.spin(dummy_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    dummy_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
