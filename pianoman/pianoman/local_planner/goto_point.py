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

from me134_interfaces.srv import PosCmdStamped

import math
import time
import sys


# PARAMETERS:
POS_L = [-0.5, -0.5, 0.5]
MOVE_TIME = 3.0

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(PosCmdStamped, 'point_cmd')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PosCmdStamped.Request()

    def send_request(self, pos_L, move_time):
        self.req.move_time.sec = math.floor(move_time)
        self.req.move_time.nanosec = max(0, math.floor(move_time - math.floor(move_time) * 1e9))
        
        self.req.goal.x = pos_L[0]
        self.req.goal.y = pos_L[1]
        self.req.goal.z = pos_L[2]

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):    
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])

    rclpy.init(args=args)

    POS_L = [x, y, z]

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(POS_L, MOVE_TIME)
    minimal_client.get_logger().info(f"Success: {response.success}")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
