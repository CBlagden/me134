"""hw5p1sol.py

   This is the solution code for HW5 Problem 1.

   This should re-use the trajectory construction from HW4 P4.

   It is also updated to properly handle shut down when the trajectory
   runs out of segments.  And provides the time step (dt) to the
   evaluation, in preparation for the coming inverse kinematics.  The
   generator node itself is pulled into a seperately file.

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
        # Pick the target.
        xgoal = np.array([0.2, -1.0, 0.5]).reshape((3, 1))

        # Build up the list of segments, starting with nothing.
        self.segments = []

        # Set the initial joint value guess.
        q = np.array([0.0, np.pi / 2, -np.pi / 2]).reshape(3, 1)

        # Newton-Raphson Algorithm: Iterate seven times.
        for k in range(7):
            x = fkin(q)
            J = Jac(q)
            qnext = q + np.linalg.inv(J) @ (xgoal - x)

            self.segments.append(GotoCubic(q, qnext, 1.0))
            q = qnext

        # Disable cyclic to end after the last segment.
        self.cyclic = False

        # Zero the start time of the current segment.
        self.t0 = 0.0

    # Declare the joint names.
    def jointnames(self):
        # Return a list of joint names FOR THE EXPECTED URDF!
        return ["theta1", "theta2", "theta3"]

    # Evaluate at the given time.  This was last called (dt) ago.
    def evaluate(self, tabsolute, dt):
        # Make sure we have a segment.
        if len(self.segments) == 0:
            return None

        # Also check whether the current segment is done.
        if self.segments[0].completed(tabsolute - self.t0):
            # If the current segment is done, shift to the next
            self.t0 = self.t0 + self.segments[0].duration()
            seg = self.segments.pop(0)
            if self.cyclic:
                self.segments.append(seg)

            # Make sure we still have something to do.
            if len(self.segments) == 0:
                return None

        # Compute the positions/velocities as a function of time.
        (q, qdot) = self.segments[0].evaluate(tabsolute - self.t0)

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
