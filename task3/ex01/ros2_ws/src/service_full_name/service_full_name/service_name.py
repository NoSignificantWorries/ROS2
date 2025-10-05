#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from service_full_name.srv import SummFullName

class FullNameService(Node):
    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(SummFullName, 'SummFullName', self.add_three_strings_callback)
        self.get_logger().info('Full Name Service is ready')

    def add_three_strings_callback(self, request, response):
        response.full_name = f"{request.last_name} {request.first_name} {request.patronymic}"
        self.get_logger().info(f'Incoming request: {request.last_name}, {request.first_name}, {request.patronymic}')
        self.get_logger().info(f'Sending response: {response.full_name}')
        return response

def main(args=None):
    rclpy.init(args=args)
    full_name_service = FullNameService()
    rclpy.spin(full_name_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
