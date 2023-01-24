# Node to test our 3 DOF robot.
# Goal: tap a specific position on the table.
#
# ME 134 LOS PENGUINOS

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from threedof_demo.KinematicChain import KinematicChain as KinematicChain
from threedof_demo.Segments import GotoCubic
import numpy as np

RATE = 20.0  # Hz
SEGMENT_T = 5


class PointAndClickNode(Node):
    def __init__(self):
        super().__init__("point_and_click")

        # PUBLISHERS
        self.pub_jtcmd = self.create_publisher(JointState, "/joint_commands", 10)

        # SUBSCRIBERS
        self.sub_jtstate = self.create_subscription(
            JointState, "/joint_states", self.cb_jtstate, 10
        )

        # TIMERS
        self.cmdtimer = self.create_timer(1 / RATE, self.sendcmd)
        self.jointnames = ["pan_joint", "middle_joint", "penguin_joint"]

        # # Set up the kinematic chain object.
        self.chain = KinematicChain("base_link", "penguin_link")

        self.q = None
        self.time = 0
        self.x_off, self.y_off, self.z_off = (
            0.4970353424430905,
            0.050472416460127276,
            0.12211428352936989,
        )

        self.sols = self.get_sols(0.510, 0.316, -0.1, 0.0, 0)
        self.curr_segment_idx = None
        self.curr_segment = None
        self.zeroed = False
        self.last_segment_start = None

    # callback for /joint_states
    def cb_jtstate(self, msg):
        self.q = msg.position

    # callback for command timer
    def sendcmd(self):
        self.time += 1 / RATE

        if self.q is not None:
            if not self.zeroed:
                self.zeroed = True
                self.last_segment_start = self.time
                self.curr_segment_idx = -1
                self.curr_segment = GotoCubic(
                    np.array(self.q), np.array([0.0, 0.0, 0.0]), SEGMENT_T
                )
            elif self.curr_segment.completed(self.time - self.last_segment_start):
                self.last_segment_start = self.time
                self.curr_segment_idx = (self.curr_segment_idx + 1) % len(self.sols)
                self.curr_segment = GotoCubic(
                    np.array(self.q),
                    np.array(self.sols[self.curr_segment_idx]),
                    SEGMENT_T,
                )

        cmdmsg = JointState()
        cmdmsg.header.stamp = self.get_clock().now().to_msg()
        cmdmsg.name = self.jointnames
        if self.curr_segment != None:
            (pos, _) = self.curr_segment.evaluate(self.time - self.last_segment_start)
            cmdmsg.position = [
                np.float64(pos[0]).item(),
                np.float64(pos[1]).item(),
                np.float64(pos[2]).item(),
            ]
        else:
            cmdmsg.position = []
        cmdmsg.position = []
        cmdmsg.effort = [0.0, 0.0, 0.0]
        self.pub_jtcmd.publish(cmdmsg)

        [ptip, Rtip, Jv, Jw] = self.chain.fkin(self.q)

    def get_sols(self, l1, l2, x, y, z):
        # offset goal position to be described in frame of middle joint
        x -= self.x_off
        y -= self.y_off
        z -= self.z_off

        R = np.sqrt(x * x + y * y)
        THETA2 = np.arccos((x * x + y * y + z * z - l1 * l1 - l2 * l2) / (2 * l1 * l2))

        def pSol(r, theta2):
            thetapan = np.arctan2(-x / r, y / r)
            theta1 = np.arctan2(z, r) - np.arctan2(
                l2 * np.sin(theta2), l1 + l2 * np.cos(theta2)
            )
            return np.array([thetapan, theta1, theta2])

        pA = pSol(R, THETA2)
        pB = pSol(-R, THETA2)
        pC = pSol(R, -THETA2)
        pD = pSol(-R, -THETA2)

        valid_sols = []
        for sol in [pA, pB, pC, pD]:
            if np.sign(sol[0] * sol[1]) == 1:
                valid_sols.append(sol)

        return valid_sols


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
