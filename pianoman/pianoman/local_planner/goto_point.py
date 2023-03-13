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
MOVE_TIME = 2.0

class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(PosCmdStamped, 'point_cmd')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = PosCmdStamped.Request()

    def send_request(self, move_time, pos_L, pos_R):
        self.req.move_time.sec = math.floor(move_time)
        self.req.move_time.nanosec = max(0, math.floor(move_time - math.floor(move_time) * 1e9))
        
        self.req.goal_left.x = pos_L[0]
        self.req.goal_left.y = pos_L[1]
        self.req.goal_left.z = pos_L[2]
        self.req.cmd_left = True

        if (pos_R):
            self.req.goal_right.x = pos_R[0]
            self.req.goal_right.y = pos_R[1]
            self.req.goal_right.z = pos_R[2]

            self.req.cmd_right = True

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


def main(args=None):

    if (len(sys.argv) == 4):
        # just command left arm
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        POS_L = [x, y, z]
        POS_R = None
    elif (len(sys.argv) == 7):
        x1 = float(sys.argv[1])
        y1 = float(sys.argv[2])
        z1 = float(sys.argv[3])
        POS_L = [x1, y1, z1]

        x2 = float(sys.argv[4])
        y2 = float(sys.argv[5])
        z2 = float(sys.argv[6])
        POS_R = [x2, y2, z2]
        

    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    response = minimal_client.send_request(MOVE_TIME, POS_L, POS_R)
    minimal_client.get_logger().info(f"Success: {response.success}")

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
