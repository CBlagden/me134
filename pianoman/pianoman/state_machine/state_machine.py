# state_machine.py
# ME 134 Team Penguinos
#
# Node to monitor and publish robot state.
# 
# SUBSCRIBES:
# TODO
# 
# PUBLISHES:
# /robot_state      me134_interfaces.msg/StateStamped

import rclpy
from rclpy.node import Node
from me134_interfaces.msg import StateStamped

import numpy as np

from pianoman.state_machine.states import State

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        # Save the state variable
        self.state = State.default()

        ## Publishers
        self.pub_state = self.create_publisher(\
                StateStamped, '/robot_state', 1)

        ## Subscribers
        # None

        # Publish the initial state
        self.pub_state.publish(self.getStateMsg())
        
    def getStateMsg(self):
        msg = StateStamped()
        msg.stamp = self.get_clock().now().to_msg()
        
        msg.state_id = self.state.value
        return msg


def main(args=None):
    # intialize ROS node.
    rclpy.init(args=args)
    node = StateMachine()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()