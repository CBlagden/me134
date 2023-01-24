# Node to test our 3 DOF robot.
# Goal: tap a specific position on the table.
# ros2 topic pub --once /pose_cmd geometry_msgs/Pose "{position: {x: -0.2, y: 0.1, z: 0.3}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
#
# ME 134 LOS PENGUINOS

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose

from threedof_demo.KinematicChain import KinematicChain as KinematicChain
import numpy as np
from threedof_demo.Segments import GotoCubic

RATE = 20.0  # Hz


class PointAndClickNode(Node):
    def __init__(self):
        super().__init__("point_and_click")

        # PUBLISHERS
        self.pub_jtcmd = self.create_publisher(JointState, "/joint_commands", 10)

        # SUBSCRIBERS
        self.sub_jtstate = self.create_subscription(
            JointState, "/joint_states", self.cb_jtstate, 10
        )
        self.sub_posecmd = self.create_subscription(
            Pose, "/pose_cmd", self.cb_posecmd, 10
        )

        # TIMERS
        self.cmdtimer = self.create_timer(1 / RATE, self.sendcmd)
        self.jointnames = ["pan_joint", "middle_joint", "penguin_joint"]

        self.movespline = None
        self.tstart = 0.0

        # Set up the kinematic chain object.
        self.chain = KinematicChain("base_link", "penguin_link")

        self.q = None
        self.qdot = None
        self.time = 0
        self.lam = 10

    # callback for /joint_states
    def cb_jtstate(self, msg):
        self.q = msg.position
        self.qdot = msg.velocity

    def cb_posecmd(self, msg):
        # only listen to pd for now
        pd = np.array([msg.position.x, msg.position.y, msg.position.z]).reshape([3, 1])
        # compute forward kinematics
        [ptip, Rtip, Jv, Jw] = self.chain.fkin(self.q)
        p = ptip

        # make spline
        self.movespline = GotoCubic(p, pd, 10)
        self.tstart = self.time

    # callback for command timer
    def sendcmd(self):
        self.time += 1 / RATE
        qdot = [0.0, 0.0, 0.0]

        if self.movespline != None:
            # exit on completion
            if self.movespline.completed(self.time - self.tstart):
                self.movespline = None
            else:
                [pd, pddot] = self.movespline.evaluate(self.time - self.tstart)
                qdot = self.goto_eepos(pd, pddot)
                qdot = qdot.reshape([3]).tolist()

        cmdmsg = JointState()
        cmdmsg.header.stamp = self.get_clock().now().to_msg()
        cmdmsg.name = self.jointnames
        cmdmsg.position = []
        cmdmsg.velocity = qdot
        cmdmsg.effort = [0.0, 0.0, 0.0]
        self.pub_jtcmd.publish(cmdmsg)

    def goto_eepos(self, pd, vd):
        # compute forward kinematics
        [ptip, Rtip, Jv, Jw] = self.chain.fkin(self.q)

        e = pd - ptip
        qdot = np.linalg.pinv(Jv) @ (vd + self.lam * e)
        return qdot


def main(args=None):
    # intialize ROS node.
    rclpy.init(args=args)
    node = PointAndClickNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
