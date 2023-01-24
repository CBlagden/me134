"""hw6p5sol.py

   This is the solution code for HW6 Problem 5.

   This explores the singularity handing while following a circle
   outside the workspace.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

"""

import rclpy
import numpy as np

from hwsols.GeneratorNode import GeneratorNode
from hwsols.KinematicChain import KinematicChain
from hwsols.TransformHelpers import *


#
#   Trajectory Class
#
class Trajectory:
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, "world", "tip", self.jointnames())

        # This should have been the starting point.
        self.q0 = np.radians(
            np.array([0, 46.5675, 0, -93.1349, 0, 0, 46.5675]).reshape((-1, 1))
        )

        # Define the various points.
        self.q0 = np.radians(
            np.array([0, 28.955, 0, -57.910, 0, 0, 0]).reshape((-1, 1))
        )
        self.p0 = np.array([0.0, 0.95, 0.6]).reshape((-1, 1))
        self.R0 = Reye()

        # Initialize the current joint position and chain data.
        self.q = self.q0
        self.chain.setjoints(self.q)

        # Also zero the task error.
        self.err = np.zeros((6, 1))

        # Pick the convergence bandwidth.
        self.lam = 20

        # Finally pick the sub-problem/part/mode
        # self.mode = 'Pseudo'          # Pseudo Inverse only
        # self.mode = 'Weighted'        # Weighted pseudo inverse
        self.mode = "Secondary"  # Secondary task as well
        node.get_logger().info("Mode = " + self.mode)

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ["theta1", "theta2", "theta3", "theta4", "theta5", "theta6", "theta7"]

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        if t > 2 * np.pi:
            return None

        # Compute the desired trajectory.
        pd = np.array([0, 0.95 - 0.25 * np.cos(t), 0.60 + 0.25 * np.sin(t)]).reshape(
            (3, 1)
        )
        vd = np.array([0, 0.25 * np.cos(t), 0.25 * np.cos(t)]).reshape((3, 1))

        Rd = Reye()
        wd = np.zeros((3, 1))

        # Grab the last joint value and task error.
        q = self.q
        err = self.err

        # Compute the inverse kinematics
        J = np.vstack((self.chain.Jv(), self.chain.Jw()))
        xdot = np.vstack((vd, wd))
        xrdot = xdot + self.lam * err

        # Part a - Pseudo Inverse only
        if self.mode == "Pseudo":
            Jinv = np.linalg.pinv(J)
            qdot = Jinv @ xrdot

        # Part b - Weighted Pseudo Inverse
        elif self.mode == "Weighted":
            gamma = 0.1
            Jinv = J.T @ np.linalg.pinv(J @ J.T + gamma**2 * np.eye(6))
            qdot = Jinv @ xrdot

        # Part c - Weighted Pseudo Inverse PLUS Secondary Task
        elif self.mode == "Secondary":
            gamma = 0.1
            Jinv = J.T @ np.linalg.pinv(J @ J.T + gamma**2 * np.eye(6))

            lams = 10.0
            qsdot = np.zeros((7, 1))
            qsdot[3, 0] = lams * (-np.pi / 2 - q[3, 0])

            qdot = Jinv @ xrdot + (np.eye(7) - Jinv @ J) @ qsdot

        else:
            raise Exception("Unknown Mode")

        # Integrate the joint position and update the kin chain data.
        q = q + dt * qdot
        self.chain.setjoints(q)

        # Compute the resulting task error (to be used next cycle).
        err = np.vstack((ep(pd, self.chain.ptip()), eR(Rd, self.chain.Rtip())))

        # Save the joint value and task error for the next cycle.
        self.q = q
        self.err = err

        # Return the position and velocity as python lists.
        return (q.flatten().tolist(), qdot.flatten().tolist())


#
#  Main Code
#
def main(args=None):
    # Initialize ROS and the generator node (100Hz) for the Trajectory.
    rclpy.init(args=args)
    generator = GeneratorNode("generator", 100, Trajectory)

    # Spin, until interrupted or the trajectory ends.
    generator.spin()

    # Shutdown the node and ROS.
    generator.shutdown()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
