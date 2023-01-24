# Node to test our 3 DOF robot.
# Goal: tap a specific position on the table.
#
# ME 134 LOS PENGUINOS

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from threedof_demo.KinematicChain import KinematicChain as KinematicChain
import numpy as np
from threedof_demo.TransformHelpers import eR

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

        # TIMERS
        self.cmdtimer = self.create_timer(1 / RATE, self.sendcmd)
        self.jointnames = ["pan_joint", "middle_joint", "penguin_joint"]

        # # Set up the kinematic chain object.
        self.chain = KinematicChain("base_link", "penguin_link")

        self.q = None
        self.qdot = None
        self.time = 0

    # callback for /joint_states
    def cb_jtstate(self, msg):
        self.q = msg.position
        self.qdot = msg.velocity

    # callback for command timer
    def sendcmd(self):
        self.time += 1 / RATE

        if self.q is None:
            return

        [ptip, Rtip, Jv, Jw] = self.chain.fkin(self.q)
        pd = np.array([0.0, 0.0, 0.4]).reshape([3, 1])

        qdot = np.array(self.qdot).reshape([3, 1])
        tau = goto_ee(pd, Rtip, ptip, Rtip, qdot, np.vstack([Jv, Jw]))
        print(tau)
        tau = tau.reshape([3])

        cmdmsg = JointState()
        cmdmsg.header.stamp = self.get_clock().now().to_msg()
        cmdmsg.name = self.jointnames
        cmdmsg.position = []
        cmdmsg.velocity = []
        cmdmsg.effort = [tau[0], tau[1], tau[2]]
        self.pub_jtcmd.publish(cmdmsg)


# CONSTANTS
KP = 15.0 * np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]).reshape([6, 1])
KV = 0.3 * np.array([1.0, 1.0, 1.0, 1.0, 1.0, 1.0]).reshape([6, 1])

# task space impedance control
def goto_ee(pd, Rd, p, R, qdot, J):
    # compute errors
    errp = pd - p
    errR = eR(Rd, R)
    err = -np.vstack([errp, errR])

    xdot = J @ qdot
    # xd_dot = kp * err - kv * pdot
    xd_dot = -(KP * err + KV * xdot)

    fd = xd_dot
    tau = J.T @ fd

    return tau


def goto_jt(l1, l2, x, y, z):
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
    print(pA, "\n", pB, "\n", pC, "\n", pD)

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
