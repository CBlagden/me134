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
import numpy as np

from pianoman.state_machine.states import State

from me134_interfaces.msg import StateStamped
from sensor_msgs.msg    import JointState

GRIP_IDX = [5, 1]
EFF_THRESHOLD = 0.5 # Nm

class StateMachine(Node):
    def __init__(self):
        super().__init__('state_machine')

        # Save the state variable
        self.state = State.default()

        ## Publishers
        self.pub_state = self.create_publisher(\
                StateStamped, '/robot_state', 1)

        ## Subscribers
        self.sub_jtstate = self.create_subscription(\
                JointState, '/joint_states', self.cb_jtstate, 10)

        # Publish the initial state
        self.pub_state.publish(self.getStateMsg())


    # Callback for /joint_states
    def cb_jtstate(self, msg):
        # Check if the effort is too high
        eff_measured = np.array(msg.effort).reshape([9,1])

        if (self.state != State.HIT_SOMETHING):
            # check for high effort
            for idx, eff in enumerate(eff_measured):
                if (idx in GRIP_IDX):
                    if (abs(eff) > 1.0):
                        # Change the state!
                        self.state = State.HIT_SOMETHING
                        self.pub_state.publish(self.getStateMsg())
                        break
                else:
                    if (abs(eff) > 0.5):
                        # Change the state!
                        self.state = State.HIT_SOMETHING
                        self.pub_state.publish(self.getStateMsg())
                        break

        # TODO: FIX THIS!!! NOT GENERAL
        elif (self.state == State.HIT_SOMETHING):
            for idx, eff in enumerate(eff_measured):
                if (idx in GRIP_IDX):
                    if (abs(eff) < 1.0):
                        # Change the state!
                        self.state = State.PLAY
                        self.pub_state.publish(self.getStateMsg())
                        break
                else:
                    if (abs(eff) < 0.2):
                        # Change the state!
                        self.state = State.PLAY
                        self.pub_state.publish(self.getStateMsg())
                        break

        
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