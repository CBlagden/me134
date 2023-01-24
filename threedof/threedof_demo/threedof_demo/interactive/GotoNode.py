# GotoNode
# Moves the robot along a cartesian or joint space spline.
# 
# Publish:
#   /joint_commands     sensor_msgs/JointState
# 
# Subscribe:
#   /joint_states       sensor_msgs/JointState
#   /pose_cmd           geometry_msgs/Point
#
# ME 134 LOS PENGUINOS

import rclpy
from rclpy.node import Node
import rclpy.time

import numpy as np

from threedof_demo.KinematicChain import KinematicChain
from threedof_demo.Segments import GotoQuintic

from sensor_msgs.msg    import JointState
from geometry_msgs.msg import Point

class GotoNode(Node):
    def __init__(self):
        super().__init__('goto_node')

        ## Class variables
        # get initial joint position sensor reading
        [self.q, self.qdot] = self.grabfbk()


        # forward kinematics
        self.chain = KinematicChain("base_link", "penguin_link")

        ## Publishers
        self.pub_jtcmd = self.create_publisher(\
                JointState, '/joint_commands', 10)

        ## Subscribers
        self.sub_jtstate = self.create_subscription(\
                JointState, '/joint_states', self.cb_jtstate, 10)
        self.sub_posecmd = self.create_subscription(\
                Point, '/point_cmd', self.cb_pointcmd, 10)


    # callback for /joint_states
    def cb_jtstate(self, msg):
        # record the current joint posiitons
        self.q = msg.position
        self.qdot = msg.velocity

    
    # callback for /pose_cmd
    def cb_pointcmd(self, msg):
        # Use this to initiate/reset splines between cartesian positions
        pd_final = list(msg)
        [pcur, _, _, _] = self.chain.fkin(self.q)
        # spline to pd_final (10 seconds)
        self.curspline = GotoQuntic(pcur, pd_final, 10)

        self.tstart = self.get_clock().now()


    # blocking function to grab a single feedback
    def grabfbk(self):
        # Create a temporary handler to grab the position.
        grabpos = None
        grabvel = None
        def cb(fbkmsg):
            grabpos = list(fbkmsg.position)
            grabvel = list(fbkmsg.velocity)
            grabready = True

        # Temporarily subscribe to get just one message.
        sub = self.create_subscription(JointState, "/joint_states", cb, 1)
        grabready = False
        while not grabready:
            rclpy.spin_once(self)
        self.destroy_subscription(sub)

        # Return the values.
        return [grabpos, grabvel]




def main(args=None):
    # intialize ROS node.
    rclpy.init(args=args)
    node = GotoNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()