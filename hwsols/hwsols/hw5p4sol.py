"""hw5p4sol.py

   This is the solution code for HW5 Problem 4.

   This creates a purely rotational movement.

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

        # Initialize the current joint position and chain data.
        self.q = np.zeros((3, 1))
        self.chain.setjoints(self.q)

        # Also zero the task error.
        self.err = np.zeros((3, 1))

        # Pick the convergence bandwidth.
        self.lam = 20

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ["pan", "tilt", "roll"]

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # Decide which phase we are in:
        if t < 2:
            # Part A (t<2).
            # Pre-compute the alpha angle.
            alpha = -np.pi / 8 * (3 * t**2 - t**3)
            alphadot = -np.pi / 8 * (6 * t - 3 * t**2)

            # Set up the desired rotation and angular velocity
            Rd = Roty(alpha)
            wd = ey() * alphadot

        else:
            # Part B (t>=2).
            # Start at RA:
            RA = Roty(-np.pi / 2)

            # Pre-compute the beta angle.
            if t < 3:
                beta = 0.5 * (t - 2) ** 2
                betadot = 1.0 * (t - 2)
            else:
                beta = 0.5 + (t - 3)
                betadot = 1.0

            # Set up the desired rotation and angular velocity
            # You can use either - they compute the same numbers.
            if False:
                # Use the diagonal axis expressed in the A frame.
                eA = exyz(0.0, np.sqrt(2) / 2, -np.sqrt(2) / 2)
                Rd = RA @ Rote(eA, beta)
                wd = RA @ eA * betadot
            else:
                # Use the diagonal axis expressed in the O frame.
                eO = exyz(np.sqrt(2) / 2, np.sqrt(2) / 2, 0.0)
                Rd = Rote(eO, beta) @ RA
                wd = eO * betadot

        # Grab the last joint value and task error.
        q = self.q
        err = self.err

        # Compute the inverse kinematics
        J = self.chain.Jw()
        qdot = np.linalg.inv(J) @ (wd + self.lam * err)

        # Integrate the joint position and update the kin chain data.
        q = q + dt * qdot
        self.chain.setjoints(q)

        # Save the joint value and precompute the task error for next cycle.
        self.q = q
        self.err = eR(Rd, self.chain.Rtip())

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
