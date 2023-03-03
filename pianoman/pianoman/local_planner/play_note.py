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
NOTE_L = "C3"
NOTE_R = "C5"
HOLD_TIME_L = 1.0
HOLD_TIME_R = 1.0
FORCE_L = 100
FORCE_R = 100

class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(NoteCmdStamped, 'note_cmd')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = NoteCmdStamped.Request()

    def send_request(self, hold_time_L, hold_time_R, note_L, note_R, force_L, force_R):
        self.req.note_left = note_L
        self.req.hold_time_left = hold_time_L
        self.req.force_left = force_L
        
        if (note_R):
            self.req.note_right = note_R
            self.req.hold_time_right = hold_time_R
            self.req.force_right = force_R

            self.req.left_only = False

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):
    if (len(sys.argv) == 2):
        NOTE_L = sys.argv[1]
        NOTE_R = None
    elif (len(sys.argv) == 3):
        NOTE_L = sys.argv[1]
        NOTE_R = sys.argv[2]

    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(HOLD_TIME_L, HOLD_TIME_R, NOTE_L, NOTE_R, FORCE_L, FORCE_R)
    minimal_client.get_logger().info(f"Success: {response.success}")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()