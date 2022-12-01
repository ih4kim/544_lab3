import sys

from interfaces.srv import StartAndEnd
from geometry_msgs.msg import Point

import rclpy
from rclpy.node import Node


class ControllerClient(Node):

    def __init__(self):
        super().__init__('controller_client')
        self.cli = self.create_client(StartAndEnd, 'start_and_end')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = StartAndEnd.Request()

    def send_request(self, start_x, start_y, end_x, end_y):
        self.req.start = Point()
        self.req.start.x = start_x
        self.req.start.y = start_y

        self.req.end = Point()
        self.req.end.x = end_x
        self.req.end.y=  end_y
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main():
    rclpy.init()

    client = ControllerClient()
    response = client.send_request(float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    if response.status:
        client.get_logger().info('Moving to destination')
    else:
        client.get_logger().info('Failed to arrive at destination; invalid destination or start point')
        
    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()