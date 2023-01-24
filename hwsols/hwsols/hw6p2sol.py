"""hw6p2sol.py

   This is the solution code for HW6 Problem 2.

   This combines position and orientation movements.  It moves (a)
   from the initial position to the starting point, then (b) up/down
   while rotating the tip (cube).

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

"""

import rclpy
import numpy as np

from hwsols.GeneratorNode import GeneratorNode
from hwsols.KinematicChain import KinematicChain
from hwsols.TransformHelpers import *


#
#   Spline Helper
#
#   We could also the Segments module.  But this seemed quicker and easier?
#
def spline(t, T, p0, pf):
    p = p0 + (pf - p0) * (3 * t**2 / T**2 - 2 * t**3 / T**3)
    v = (pf - p0) * (6 * t / T**2 - 6 * t**2 / T**3)
    return (p, v)


#
#   Trajectory Class
#
class Trajectory:
    # Initialization.
    def __init__(self, node):
        # Set up the kinematic chain object.
        self.chain = KinematicChain(node, "world", "tip", self.jointnames())

        # Define the various points.
        self.q0 = np.radians(np.array([0, 90, -90, 0, 0, 0]).reshape((-1, 1)))
        self.p0 = np.array([0.0, 0.55, 1.0]).reshape((-1, 1))
        self.R0 = Reye()

        self.plow = np.array([0.0, 0.5, 0.3]).reshape((-1, 1))
        self.phigh = np.array([0.0, 0.5, 0.9]).reshape((-1, 1))

        # Initialize the current joint position and chain data.
        self.q = self.q0
        self.chain.setjoints(self.q)

        # Also zero the task error.
        self.err = np.zeros((6, 1))

        # Pick the convergence bandwidth.
        self.lam = 20

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ["theta1", "theta2", "theta3", "theta4", "theta5", "theta6"]

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, t, dt):
        # Decide which phase we are in:
        if t < 3:
            # Approach movement:
            (s0, s0dot) = spline(t, 3, 0, 1)

            pd = self.p0 + (self.plow - self.p0) * s0
            vd = (self.plow - self.p0) * s0dot

            Rd = Rotz(-np.pi / 2 * s0)
            wd = ez() * (-np.pi / 2 * s0dot)

        else:
            # Cyclic movements, after the first 3s, repeat every 8s.
            t1 = (t - 3) % 8.0

            # Pre-compute the path variables.  To show different
            # options, we split into position/orientation path
            # variable and compute using both a sinusoid and splines.
            sp = -np.cos(0.5 * np.pi * t1)
            spdot = 0.5 * np.pi * np.sin(0.5 * np.pi * t1)

            if t1 < 4:
                (sR, sRdot) = spline(t1, 4, -1, 1)
            else:
                (sR, sRdot) = spline(t1 - 4, 4, 1, -1)

            # Use the path variables to compute the trajectory.
            pd = 0.5 * (self.phigh + self.plow) + 0.5 * (self.phigh - self.plow) * sp
            vd = +0.5 * (self.phigh - self.plow) * spdot

            Rd = Rotz(np.pi / 2 * sR)
            wd = ez() * (np.pi / 2 * sRdot)

        # Grab the last joint value and task error.
        q = self.q
        err = self.err

        # Compute the inverse kinematics
        J = np.vstack((self.chain.Jv(), self.chain.Jw()))
        xdot = np.vstack((vd, wd))
        qdot = np.linalg.inv(J) @ (xdot + self.lam * err)

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
