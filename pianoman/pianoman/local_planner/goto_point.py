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

from me134_interfaces.msg import PosCmdStamped

import math
import time

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('point_command_publisher')

    qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )
    publisher = node.create_publisher(PosCmdStamped, '/point_cmd', qos_profile)

    msg = PosCmdStamped()

    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = "base_link"

    msg.goal.x = 0.3
    msg.goal.y = 0.0
    msg.goal.z = 0.2

    move_time = 10.0

    msg.move_time.sec = math.floor(move_time)
    msg.move_time.nanosec = max(0, math.floor(move_time - math.floor(move_time) * 1e9))

    # Publish!
    publisher.publish(msg)

    time.sleep(1.0)

    # def timer_callback():
    #     msg.data = 'ObiWan Kenobi, please help me. You\'re my only hope'
    #     publisher.publish(msg)

    # timer_period = 0.5  # seconds
    # timer = node.create_timer(timer_period, timer_callback)

    # rclpy.spin(node)
    # node.destroy_timer(timer)


    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()