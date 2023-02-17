# goto_point.py
# ME 134 Team Penguinos
#
# Simple node to direct the robot to a position
# 
# PUBLISHES:
# /point_cmd        me134_interfaces.msg/PosCmdStamped

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy

from me134_interfaces.srv import NoteCmdStamped

import math
import time
import sys


# PARAMETERS:
NOTE = "C4"
HOLD_TIME = 0.5
FORCE = 100

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(NoteCmdStamped, 'note_cmd')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = NoteCmdStamped.Request()

    def send_request(self, note, hold_time, force):
        self.req.note = note
        self.req.hold_time = hold_time
        self.req.force = force

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):    
    NOTE = sys.argv[1]

    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(NOTE, HOLD_TIME, FORCE)
    minimal_client.get_logger().info(f"Success: {response.success}")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()