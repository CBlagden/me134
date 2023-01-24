"""hw5p2sol.py

   This is the solution code for HW5 Problem 2.

   This moves the tip in a straight line (tip splne), then returns in
   a joint spline.  Note we do not pre-compute a list of segment.
   Instead we create the necessary spline segment on the fly.

   Node:        /generator
   Publish:     /joint_states           sensor_msgs/JointState

"""

import rclpy
import numpy as np

from hwsols.GeneratorNode import GeneratorNode
from hwsols.Segments import Hold, Stay, GotoCubic, SplineCubic
from hwsols.hw4p3sol import fkin, Jac


#
#   Trajectory Class
#
class Trajectory:
    # Initialization.
    def __init__(self, node):
        # Define the known tip/joint positions.
        self.qA = np.radians(np.array([0, 60, -120]).reshape(3, 1))
        self.xB = np.array([0.5, -0.5, 1.0]).reshape(3, 1)

        # Pre-compute what we can.
        self.xA = fkin(self.qA)
        # Better to ensure matching than hard-coding
        # self.xA = np.array([0.0,  1.0, 0.0]).reshape(3,1)

        # qB will be determined by the ikin at runtime...

        # Select the segment duration.
        self.T = 3.0

        # Define the current segment.  The first segment will be the
        # task-space tip movement from xA to xB.  Zero the start time.
        self.t0 = 0.0
        self.segment = GotoCubic(self.xA, self.xB, self.T, space="Tip")

        # Initialize the current joint location and the matching tip
        # error (which should be zero).
        self.q = self.qA
        self.err = self.xA - fkin(self.qA)

        # Pick the convergence bandwidth.
        self.lam = 10

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ["theta1", "theta2", "theta3"]

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, tabsolute, dt):
        # Take action, based on whether the current segment's space.
        if self.segment.space() == "Tip":
            # If the movement is completed, putting us at point B, add
            # a joint move back to qA.  Re-evaluate on new segment.
            if self.segment.completed(tabsolute - self.t0):
                self.t0 = self.t0 + self.segment.duration()
                self.segment = GotoCubic(self.q, self.qA, self.T, space="Joint")
                return self.evaluate(tabsolute, dt)

            # If the movement is ongoing, compute the current values.
            (xd, xddot) = self.segment.evaluate(tabsolute - self.t0)

            # Grab the last joint value and tip error.
            q = self.q
            err = self.err

            # Compute the inverse kinematics
            J = Jac(q)
            qdot = np.linalg.inv(J) @ (xddot + self.lam * err)
            q = q + dt * qdot

            # Save the joint value and precompute the tip error for next cycle.
            self.q = q
            self.err = xd - fkin(q)

        # For joint splines:
        else:
            # If the movement is completed, putting us at point A, add
            # a new straight line to xB.  Re-evaluate on new segment.
            if self.segment.completed(tabsolute - self.t0):
                self.t0 = self.t0 + self.segment.duration()
                self.segment = GotoCubic(self.xA, self.xB, self.T, space="Tip")
                # Return None to stop.
                return self.evaluate(tabsolute, dt)

            # If the movement is ongoing, compute the current values.
            (q, qdot) = self.segment.evaluate(tabsolute - self.t0)

            # Also, to transition back to tip space, save the joint
            # value and declare a zero tip error.
            self.q = q
            self.e = np.array([0.0, 0.0, 0.0]).reshape(3, 1)

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
