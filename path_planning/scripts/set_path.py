#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node

from hippo_msgs.srv import SetPath


class SetPathNode(Node):

    def __init__(self, node_name: str) -> None:
        super().__init__(node_name=node_name)
        self.client = self.create_client(SetPath, 'carrot_controller/set_path')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service...')
        self.request = SetPath.Request()

    def send_request(self, path):
        waypoints = [
            Point(x=1.0, y=1.0, z=-1.0),
            Point(x=3.0, y=2.0, z=-1.0),
            Point(x=5.0, y=1.0, z=-1.0)
        ]
        self.request.path.waypoints = waypoints
        self.request.path.is_loop = True
        self.future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()
    node = SetPathNode('path_setter')
    node.get_logger().info(f'{node.send_request("test")}')
    node.get_logger().info(f'{node.send_request("test")}')
    node.get_logger().info(f'{node.send_request("test")}')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
