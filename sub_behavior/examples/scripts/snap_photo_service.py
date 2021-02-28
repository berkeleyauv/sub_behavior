#!/usr/bin/env python3

from sub_behavior_interfaces.srv import SnapPhoto

import rclpy
from rclpy.node import Node
import time


class SnapPhotoService(Node):

    def __init__(self):
        super().__init__('snap_photo_service')
        self.srv = self.create_service(SnapPhoto, 'snap_photo', self.snap_photo_callback)

    def snap_photo_callback(self, request, response):
        self.get_logger().info(f'Incoming request\nimage_topic: ${request.image_topic}')
        response.path = '/this path does not exist yet'
        time.sleep(2)
        return response


def main(args=None):
    rclpy.init(args=args)
    snap_photo_service = SnapPhotoService()
    rclpy.spin(snap_photo_service)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
